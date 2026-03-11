#!/usr/bin/env python3
"""
Read current date/time from a Generac Evolution controller over Modbus RTU.

Based on jgyates/genmon data/controller/Evolution_Liquid_Cooled.json:
  0x000E = Generator Time      (hour in high byte, minute in low byte)
  0x000F = Generator Date      (month in high byte, day in low byte)
  0x0010 = Generator Day/Year  (day-of-week in high byte, year since 2000 in low byte)

Defaults follow Genmon's common settings for Evolution controllers:
  - port: /dev/USB232
  - baud: 9600
  - 8N1
  - slave address: 0x9D
  - function code: 0x03 (read holding registers)
"""

import argparse
import datetime as dt
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial\nInstall with: pip install pyserial", file=sys.stderr)
    sys.exit(2)


DAY_NAMES = [
    "Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday"
]


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_read_holding_request(slave: int, start_reg: int, quantity: int) -> bytes:
    payload = bytes([
        slave & 0xFF,
        0x03,
        (start_reg >> 8) & 0xFF,
        start_reg & 0xFF,
        (quantity >> 8) & 0xFF,
        quantity & 0xFF,
    ])
    crc = modbus_crc(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def parse_read_holding_response(resp: bytes, slave: int, quantity: int):
    expected_len = 5 + (quantity * 2)
    if len(resp) != expected_len:
        raise ValueError(f"Unexpected response length {len(resp)} (expected {expected_len})")

    body = resp[:-2]
    rx_crc = resp[-2] | (resp[-1] << 8)
    calc_crc = modbus_crc(body)
    if rx_crc != calc_crc:
        raise ValueError(f"CRC mismatch: received 0x{rx_crc:04X}, calculated 0x{calc_crc:04X}")

    if body[0] != (slave & 0xFF):
        raise ValueError(f"Unexpected slave address: 0x{body[0]:02X}")

    func = body[1]
    if func & 0x80:
        if len(body) >= 3:
            raise ValueError(f"Modbus exception 0x{body[2]:02X}")
        raise ValueError("Modbus exception response")
    if func != 0x03:
        raise ValueError(f"Unexpected function code: 0x{func:02X}")

    byte_count = body[2]
    if byte_count != quantity * 2:
        raise ValueError(f"Unexpected byte count {byte_count}")

    regs = []
    data = body[3:]
    for i in range(0, len(data), 2):
        regs.append((data[i] << 8) | data[i + 1])
    return regs


def decode_evolution_datetime(reg_000e: int, reg_000f: int, reg_0010: int):
    hour = (reg_000e >> 8) & 0xFF
    minute = reg_000e & 0xFF

    month = (reg_000f >> 8) & 0xFF
    day = reg_000f & 0xFF

    dow = (reg_0010 >> 8) & 0xFF
    year = 2000 + (reg_0010 & 0xFF)

    day_name = DAY_NAMES[dow] if 0 <= dow < len(DAY_NAMES) else f"Unknown({dow})"

    controller_dt = dt.datetime(year, month, day, hour, minute)
    return controller_dt, day_name, {
        "reg_000e": reg_000e,
        "reg_000f": reg_000f,
        "reg_0010": reg_0010,
        "hour": hour,
        "minute": minute,
        "month": month,
        "day": day,
        "year": year,
        "day_of_week_index": dow,
        "day_of_week_name": day_name,
    }


def read_registers(port: str, slave: int, start_reg: int, quantity: int, baudrate: int, timeout: float):
    req = build_read_holding_request(slave, start_reg, quantity)
    expected_len = 5 + (quantity * 2)

    with serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
    ) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(req)
        ser.flush()
        # Small pause helps some USB/RS485 adapters settle.
        time.sleep(0.05)
        resp = ser.read(expected_len)

    return parse_read_holding_response(resp, slave, quantity)


def main():
    parser = argparse.ArgumentParser(description="Read date/time from an Evolution controller")
    parser.add_argument("--port", default="/dev/USB232", help="Serial port (default: /dev/USB232)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--slave", default="0x9D", help="Modbus slave address, decimal or hex (default: 0x9D)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout seconds (default: 1.0)")
    parser.add_argument("--raw", action="store_true", help="Also print raw register values")
    args = parser.parse_args()

    slave = int(str(args.slave), 0)

    try:
        regs = read_registers(args.port, slave, start_reg=0x000E, quantity=3, baudrate=args.baud, timeout=args.timeout)
        controller_dt, day_name, details = decode_evolution_datetime(*regs)
    except Exception as e:
        print(f"Error reading controller time: {e}", file=sys.stderr)
        sys.exit(1)

    print(controller_dt.strftime(f"%Y-%m-%d %H:%M") + f" ({day_name})")
    if args.raw:
        print(f"0x000E = 0x{details['reg_000e']:04X}")
        print(f"0x000F = 0x{details['reg_000f']:04X}")
        print(f"0x0010 = 0x{details['reg_0010']:04X}")
        print(details)


if __name__ == "__main__":
    main()
