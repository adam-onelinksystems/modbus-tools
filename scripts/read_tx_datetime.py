#!/usr/bin/env python3
"""
Read or write the current date/time on a TX transfer switch controller over Modbus RTU.

RTC block discovered from field scans:
  200 = packed day/month    (high byte = day, low byte = month)
  201 = packed year/weekday (high byte = 2-digit year, low byte = weekday code)
  202 = packed hour/minute  (high byte = hour, low byte = minute)
  203 = seconds

Examples:
  python3 read_tx_datetime.py
  python3 read_tx_datetime.py --port /dev/tty.usbserial-FT4WX7ID --slave 240
  python3 read_tx_datetime.py --set 2031-12-31T13:09:05 --weekday 4
"""

import argparse
import datetime as dt
import sys

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial\nInstall with: pip install pyserial", file=sys.stderr)
    sys.exit(2)


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_request(slave: int, function_code: int, start_reg: int, quantity: int) -> bytes:
    payload = bytes([
        slave & 0xFF,
        function_code & 0xFF,
        (start_reg >> 8) & 0xFF,
        start_reg & 0xFF,
        (quantity >> 8) & 0xFF,
        quantity & 0xFF,
    ])
    crc = modbus_crc(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_write_single_request(slave: int, register: int, value: int) -> bytes:
    payload = bytes([
        slave & 0xFF,
        0x06,
        (register >> 8) & 0xFF,
        register & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ])
    crc = modbus_crc(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def parse_response(resp: bytes, slave: int, function_code: int, quantity: int):
    expected_len = 5 + quantity * 2
    if len(resp) != expected_len:
        raise ValueError(f"short/long response: got {len(resp)} bytes expected {expected_len}")

    body = resp[:-2]
    rx_crc = resp[-2] | (resp[-1] << 8)
    calc_crc = modbus_crc(body)
    if rx_crc != calc_crc:
        raise ValueError(f"crc mismatch rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}")

    if body[0] != (slave & 0xFF):
        raise ValueError(f"unexpected slave 0x{body[0]:02X}")

    fc = body[1]
    if fc & 0x80:
        code = body[2] if len(body) > 2 else None
        raise ValueError(f"modbus exception fc=0x{fc:02X} code=0x{code:02X}")
    if fc != function_code:
        raise ValueError(f"unexpected function code 0x{fc:02X}")

    byte_count = body[2]
    if byte_count != quantity * 2:
        raise ValueError(f"unexpected byte count {byte_count}")

    data = body[3:]
    regs = []
    for i in range(0, len(data), 2):
        regs.append((data[i] << 8) | data[i + 1])
    return regs


def parse_write_response(resp: bytes, slave: int, register: int, value: int):
    expected_len = 8
    if len(resp) != expected_len:
        raise ValueError(f"short/long write response: got {len(resp)} bytes expected {expected_len}")
    body = resp[:-2]
    rx_crc = resp[-2] | (resp[-1] << 8)
    calc_crc = modbus_crc(body)
    if rx_crc != calc_crc:
        raise ValueError(f"write crc mismatch rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}")
    if body[0] != (slave & 0xFF):
        raise ValueError(f"unexpected write slave 0x{body[0]:02X}")
    if body[1] & 0x80:
        code = body[2] if len(body) > 2 else None
        raise ValueError(f"write modbus exception fc=0x{body[1]:02X} code=0x{code:02X}")
    echoed_reg = (body[2] << 8) | body[3]
    echoed_val = (body[4] << 8) | body[5]
    if echoed_reg != register or echoed_val != value:
        raise ValueError(f"write echo mismatch reg=0x{echoed_reg:04X} val=0x{echoed_val:04X}")


def read_regs(port: str, baud: int, slave: int, start_reg: int, count: int, timeout: float):
    req = build_request(slave, 3, start_reg, count)
    expected_len = 5 + count * 2
    with serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
    ) as ser:
        ser.reset_input_buffer()
        ser.write(req)
        ser.flush()
        resp = ser.read(expected_len)
    return parse_response(resp, slave, 3, count)


def write_single(ser, slave: int, register: int, value: int):
    req = build_write_single_request(slave, register, value)
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    resp = ser.read(8)
    parse_write_response(resp, slave, register, value)


def decode_regs(reg200: int, reg201: int, reg202: int, reg203: int):
    day = (reg200 >> 8) & 0xFF
    month = reg200 & 0xFF
    year = (reg201 >> 8) & 0xFF
    weekday_code = reg201 & 0xFF
    hour = (reg202 >> 8) & 0xFF
    minute = reg202 & 0xFF
    second = reg203 & 0xFFFF
    return day, month, year, weekday_code, hour, minute, second


def encode_regs(dt_obj: dt.datetime, weekday_code: int):
    reg200 = ((dt_obj.day & 0xFF) << 8) | (dt_obj.month & 0xFF)
    reg201 = (((dt_obj.year % 100) & 0xFF) << 8) | (weekday_code & 0xFF)
    reg202 = ((dt_obj.hour & 0xFF) << 8) | (dt_obj.minute & 0xFF)
    reg203 = dt_obj.second & 0xFFFF
    return reg200, reg201, reg202, reg203


def main():
    parser = argparse.ArgumentParser(description="Read or write TX transfer switch RTC date/time")
    parser.add_argument("--port", default="/dev/tty.usbserial-FT4WX7ID")
    parser.add_argument("--slave", default="240", help="decimal or hex slave id")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--set", dest="set_datetime", help="Set controller datetime as YYYY-MM-DDTHH:MM:SS")
    parser.add_argument("--weekday", type=int, help="Weekday code low byte for register 201 (required with --set)")
    args = parser.parse_args()

    slave = int(str(args.slave), 0)

    if args.set_datetime:
        if args.weekday is None:
            raise SystemExit("--weekday is required with --set until weekday encoding is fully confirmed")
        new_dt = dt.datetime.fromisoformat(args.set_datetime)
        reg200, reg201, reg202, reg203 = encode_regs(new_dt, args.weekday)
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
        ) as ser:
            write_single(ser, slave, 200, reg200)
            write_single(ser, slave, 201, reg201)
            write_single(ser, slave, 202, reg202)
            write_single(ser, slave, 203, reg203)
        print(f"Wrote registers: 200=0x{reg200:04X} 201=0x{reg201:04X} 202=0x{reg202:04X} 203=0x{reg203:04X}")

    reg200, reg201, reg202, reg203 = read_regs(args.port, args.baud, slave, 200, 4, args.timeout)
    day, month, year, weekday_code, hour, minute, second = decode_regs(reg200, reg201, reg202, reg203)

    print(f"Registers: 200=0x{reg200:04X} 201=0x{reg201:04X} 202=0x{reg202:04X} 203=0x{reg203:04X}")
    print(f"Decoded date: 20{year:02d}-{month:02d}-{day:02d} (weekday code {weekday_code})")
    print(f"Decoded time: {hour:02d}:{minute:02d}:{second:02d}")


if __name__ == "__main__":
    main()
