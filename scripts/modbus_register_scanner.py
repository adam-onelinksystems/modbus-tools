#!/usr/bin/env python3
"""
Read-only Modbus RTU register scanner / poller.

Default target matches the latest requested TX switch test settings:
  - port:  /dev/tty.usbserial-FT4WX7ID
  - slave: 240 (0xF0)

Features:
  - Scan holding registers (FC3) or input registers (FC4)
  - Read in chunks so large scans are safer
  - Optional repeat polling
  - Optional show-only-changes mode
  - CSV output for later analysis

Examples:
  python3 modbus_register_scanner.py
  python3 modbus_register_scanner.py --fc 4 --start 0 --count 300
  python3 modbus_register_scanner.py --interval 2 --watch --csv scan.csv
"""

import argparse
import csv
import datetime as dt
import sys
import time

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


def read_chunk(ser, slave: int, function_code: int, start_reg: int, quantity: int):
    req = build_request(slave, function_code, start_reg, quantity)
    expected_len = 5 + quantity * 2
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    time.sleep(0.03)
    resp = ser.read(expected_len)
    return parse_response(resp, slave, function_code, quantity)


def write_single_register(ser, slave: int, register: int, value: int, allow_value_mismatch: bool = False):
    req = build_write_single_request(slave, register, value)
    expected_len = 8
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    time.sleep(0.03)
    resp = ser.read(expected_len)
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
    if body[1] != 0x06:
        raise ValueError(f"unexpected write function code 0x{body[1]:02X}")
    echoed_reg = (body[2] << 8) | body[3]
    echoed_val = (body[4] << 8) | body[5]
    if echoed_reg != register:
        raise ValueError(f"write echo register mismatch reg=0x{echoed_reg:04X}")
    if (not allow_value_mismatch) and echoed_val != value:
        raise ValueError(f"write echo mismatch reg=0x{echoed_reg:04X} val={echoed_val}")
    return echoed_reg, echoed_val


def scan_registers(port, baud, slave, function_code, start, count, chunk_size, timeout, pause, unlock_register=None, unlock_value=None, verify_register=None):
    result = {}
    errors = {}
    total_chunks = (count + chunk_size - 1) // chunk_size
    processed_registers = 0
    with serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
    ) as ser:
        if unlock_register is not None and unlock_value is not None:
            print(f"Unlocking controller: write {unlock_value} to register {unlock_register}...", flush=True)
            echoed_reg, echoed_val = write_single_register(ser, slave, unlock_register, unlock_value, allow_value_mismatch=True)
            if echoed_val == unlock_value:
                print("  Unlock write acknowledged with matching echo", flush=True)
            else:
                print(f"  Unlock write acknowledged with controller response value {echoed_val} (0x{echoed_val:04X})", flush=True)
            if verify_register is not None:
                try:
                    verify_val = read_chunk(ser, slave, function_code, verify_register, 1)[0]
                    print(f"  Verify register {verify_register} = {verify_val} (0x{verify_val:04X})", flush=True)
                except Exception as e:
                    print(f"  Verify read failed for register {verify_register}: {e}", flush=True)
            time.sleep(pause)
        reg = start
        remaining = count
        chunk_index = 0
        while remaining > 0:
            qty = min(chunk_size, remaining)
            chunk_index += 1
            chunk_start = reg
            chunk_end = reg + qty - 1
            print(f"Reading {chunk_start}-{chunk_end} (chunk {chunk_index}/{total_chunks})...", flush=True)
            try:
                values = read_chunk(ser, slave, function_code, reg, qty)
                for i, value in enumerate(values):
                    result[reg + i] = value
                processed_registers += qty
                pct = (processed_registers / count) * 100 if count else 100
                print(f"  OK: {qty} regs read | progress {processed_registers}/{count} ({pct:.1f}%)", flush=True)
            except Exception as e:
                print(f"  Chunk read failed: {e} | falling back to single-register reads", flush=True)
                for single in range(reg, reg + qty):
                    try:
                        values = read_chunk(ser, slave, function_code, single, 1)
                        result[single] = values[0]
                        processed_registers += 1
                        pct = (processed_registers / count) * 100 if count else 100
                        print(f"    OK reg {single}: {values[0]} (0x{values[0]:04X}) | progress {processed_registers}/{count} ({pct:.1f}%)", flush=True)
                    except Exception as e2:
                        errors[single] = str(e2)
                        processed_registers += 1
                        pct = (processed_registers / count) * 100 if count else 100
                        print(f"    SKIP reg {single}: {e2} | progress {processed_registers}/{count} ({pct:.1f}%)", flush=True)
                    time.sleep(pause)
            reg += qty
            remaining -= qty
            time.sleep(pause)
    return result, errors


def print_snapshot(snapshot, previous=None, changes_only=False):
    for reg in sorted(snapshot.keys()):
        value = snapshot[reg]
        old = None if previous is None else previous.get(reg)
        changed = previous is None or old != value
        if changes_only and not changed:
            continue
        if previous is None:
            print(f"{reg:05d}  0x{reg:04X}  {value:5d}  0x{value:04X}")
        else:
            marker = "*" if changed else " "
            print(f"{marker} {reg:05d}  0x{reg:04X}  {value:5d}  0x{value:04X}" + (f"  (was {old})" if changed and old is not None else ""))


def append_csv(csv_path, poll_num, function_code, snapshot):
    ts = dt.datetime.now().isoformat()
    write_header = False
    try:
        with open(csv_path, 'r', newline=''):
            pass
    except FileNotFoundError:
        write_header = True

    with open(csv_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["timestamp", "poll", "fc", "register_dec", "register_hex", "value_dec", "value_hex"])
        for reg in sorted(snapshot.keys()):
            value = snapshot[reg]
            writer.writerow([ts, poll_num, function_code, reg, f"0x{reg:04X}", value, f"0x{value:04X}"])


def main():
    parser = argparse.ArgumentParser(description="Read-only Modbus RTU register scanner")
    parser.add_argument("--port", default="/dev/tty.usbserial-FT4WX7ID")
    parser.add_argument("--slave", default="240", help="decimal or hex slave id, default 240")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--fc", type=int, choices=[3, 4], default=3, help="3=holding, 4=input")
    parser.add_argument("--start", type=int, default=500, help="start register")
    parser.add_argument("--count", type=int, default=300, help="number of registers to scan")
    parser.add_argument("--chunk", type=int, default=20, help="registers per chunk read")
    parser.add_argument("--timeout", type=float, default=0.7)
    parser.add_argument("--pause", type=float, default=0.03, help="pause between reads")
    parser.add_argument("--interval", type=float, default=0.0, help="repeat poll interval seconds; 0 = single scan")
    parser.add_argument("--watch", action="store_true", help="on repeated polls, print only changed values after first pass")
    parser.add_argument("--csv", help="append results to CSV file")
    parser.add_argument("--show-errors", action="store_true")
    parser.add_argument("--unlock-register", type=int, default=100, help="write-single register used for unlock/security")
    parser.add_argument("--unlock-value", type=int, default=7315, help="unlock/security code to write before reads")
    parser.add_argument("--verify-register", type=int, default=101, help="optional register to read after unlock")
    parser.add_argument("--no-unlock", action="store_true", help="skip the initial unlock write")
    args = parser.parse_args()

    slave = int(str(args.slave), 0)
    previous = None
    poll_num = 0

    while True:
        poll_num += 1
        print(f"\n=== poll {poll_num} @ {dt.datetime.now().isoformat()} fc={args.fc} slave={slave} port={args.port} range={args.start}-{args.start + args.count - 1} ===")
        snapshot, errors = scan_registers(
            port=args.port,
            baud=args.baud,
            slave=slave,
            function_code=args.fc,
            start=args.start,
            count=args.count,
            chunk_size=args.chunk,
            timeout=args.timeout,
            pause=args.pause,
            unlock_register=(None if args.no_unlock else args.unlock_register),
            unlock_value=(None if args.no_unlock else args.unlock_value),
            verify_register=(None if args.no_unlock else args.verify_register),
        )

        print_snapshot(snapshot, previous=previous, changes_only=(args.watch and previous is not None))
        print(f"\nRead {len(snapshot)} registers; {len(errors)} errors")

        if args.show_errors and errors:
            for reg in sorted(errors.keys()):
                print(f"ERR {reg:05d} 0x{reg:04X} {errors[reg]}")

        if args.csv:
            append_csv(args.csv, poll_num, args.fc, snapshot)

        previous = snapshot

        if args.interval <= 0:
            break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
