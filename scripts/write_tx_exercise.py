#!/usr/bin/env python3
"""
Read or write TX transfer switch exercise schedule settings over Modbus RTU.

Known exercise registers:
  650 = Exercise Seconds
  651 = Exercise Type
  652 = Exercise Day
  653 = Exercise Hour
  654 = Exercise Minute
  655 = Exercise Duration
  656 = Exercise Xfer

Writes use the controller unlock register first:
  100 = security/unlock register
  value 3219 = confirmed unlock code for this controller

Examples:
  python3 write_tx_exercise.py
  python3 write_tx_exercise.py --set --type 1 --day 3 --hour 9 --minute 0 --duration 20 --xfer 1
  python3 write_tx_exercise.py --set --type 3 --day 15 --hour 14 --minute 30 --duration 10 --xfer 0
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial\nInstall with: pip install pyserial", file=sys.stderr)
    sys.exit(2)

UNLOCK_REGISTER = 100
UNLOCK_VALUE = 3219

EXERCISE_TYPE_MAP = {
    0: "Daily",
    1: "Weekly",
    2: "Bi Weekly",
    3: "Monthly",
    4: "First Week",
    5: "Second Week",
    6: "Third Week",
    7: "Fourth Week",
}


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


def parse_write_response(resp: bytes, slave: int, register: int, value: int, allow_value_mismatch: bool = False):
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
    if echoed_reg != register:
        raise ValueError(f"write echo register mismatch reg=0x{echoed_reg:04X}")
    if (not allow_value_mismatch) and echoed_val != value:
        raise ValueError(f"write echo mismatch reg=0x{echoed_reg:04X} val=0x{echoed_val:04X}")
    return echoed_reg, echoed_val


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


def read_reg(ser, slave: int, register: int):
    req = build_request(slave, 3, register, 1)
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    resp = ser.read(7)
    return parse_response(resp, slave, 3, 1)[0]


def read_exercise_regs(port: str, baud: int, slave: int, timeout: float):
    try:
        return read_regs(port, baud, slave, 650, 7, timeout)
    except Exception as e:
        print(f"Block read 650-656 failed: {e} | falling back to single-register reads")
        values = []
        with serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        ) as ser:
            for register in range(650, 657):
                values.append(read_reg(ser, slave, register))
        return values


def write_single(ser, slave: int, register: int, value: int, allow_value_mismatch: bool = False):
    req = build_write_single_request(slave, register, value)
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    resp = ser.read(8)
    return parse_write_response(resp, slave, register, value, allow_value_mismatch=allow_value_mismatch)


def print_schedule(values):
    seconds, ex_type, day, hour, minute, duration, xfer = values
    type_name = EXERCISE_TYPE_MAP.get(ex_type, f"Unknown({ex_type})")
    print(f"650 Exercise Seconds : {seconds}")
    print(f"651 Exercise Type    : {ex_type} ({type_name})")
    print(f"652 Exercise Day     : {day}")
    print(f"653 Exercise Hour    : {hour:02d}")
    print(f"654 Exercise Minute  : {minute:02d}")
    print(f"655 Exercise Duration: {duration}")
    print(f"656 Exercise Xfer    : {xfer}")


def main():
    parser = argparse.ArgumentParser(description="Read or write TX transfer switch exercise schedule")
    parser.add_argument("--port", default="/dev/tty.usbserial-FT4WX7ID")
    parser.add_argument("--slave", default="240", help="decimal or hex slave id")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--set", action="store_true", help="write exercise settings")
    parser.add_argument("--type", type=int, choices=range(0, 8), help="exercise type (0-7)")
    parser.add_argument("--day", type=int, help="exercise day/date value for reg 652")
    parser.add_argument("--hour", type=int, help="exercise hour for reg 653")
    parser.add_argument("--minute", type=int, help="exercise minute for reg 654")
    parser.add_argument("--duration", type=int, help="exercise duration for reg 655")
    parser.add_argument("--xfer", type=int, choices=[0, 1], help="exercise transfer flag for reg 656")
    args = parser.parse_args()

    slave = int(str(args.slave), 0)

    if args.set:
        current = read_exercise_regs(args.port, args.baud, slave, args.timeout)
        new_values = [
            current[0],
            args.type if args.type is not None else current[1],
            args.day if args.day is not None else current[2],
            args.hour if args.hour is not None else current[3],
            args.minute if args.minute is not None else current[4],
            args.duration if args.duration is not None else current[5],
            args.xfer if args.xfer is not None else current[6],
        ]

        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
        ) as ser:
            _, unlock_echo = write_single(ser, slave, UNLOCK_REGISTER, UNLOCK_VALUE, allow_value_mismatch=True)
            print(f"Unlock write sent to reg {UNLOCK_REGISTER}; controller echoed value 0x{unlock_echo:04X}")
            for register, value in zip(range(650, 657), new_values):
                _, echoed_val = write_single(ser, slave, register, value, allow_value_mismatch=True)
                if echoed_val == value:
                    print(f"Wrote reg {register} = {value} (matching echo)")
                else:
                    print(f"Wrote reg {register} = {value} (controller echoed {echoed_val})")
        print("Waiting 1 second before verification...")
        time.sleep(1.0)

    values = read_exercise_regs(args.port, args.baud, slave, args.timeout)
    print_schedule(values)


if __name__ == "__main__":
    main()
