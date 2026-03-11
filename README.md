# modbus-tools

Small Modbus utilities for controller discovery and polling.

## Files

### scripts/
- `modbus_register_scanner.py`
  - Read-only Modbus RTU scanner/poller
  - Optional unlock write before polling
  - Progress output for each read/chunk
  - CSV logging
  - Skip unreadable registers and continue

- `read_evolution_datetime.py`
  - Reads current date/time from a Generac Evolution controller
  - Based on the Genmon Evolution Liquid Cooled register map

- `read_tx_datetime.py`
  - Reads or writes current RTC time/date fields on a TX transfer switch controller
  - Uses discovered RTC registers 200-203

- `write_tx_exercise.py`
  - Reads or writes TX exercise schedule settings
  - Uses exercise registers 650-656 with unlock-first write flow

### docs/
- `tx_transfer_switch_registers.csv`
  - Collected TX transfer switch register map notes from field documentation/screenshots

## Current scanner defaults

- Port: `/dev/tty.usbserial-FT4WX7ID`
- Slave: `240`
- Start register: `500`
- Count: `300`
- Unlock register: `100`
- Unlock value: `3219`
- Verify register: `101`

## Example usage

```bash
python3 scripts/modbus_register_scanner.py --interval 2 --watch --csv scan.csv
python3 scripts/read_evolution_datetime.py
python3 scripts/read_tx_datetime.py
python3 scripts/read_tx_datetime.py --set 2031-03-15T13:20:11 --weekday 7
python3 scripts/write_tx_exercise.py --set --type 1 --day 3 --hour 9 --minute 0 --duration 20 --xfer 1
```
