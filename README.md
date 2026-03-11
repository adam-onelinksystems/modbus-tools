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

### docs/
- `tx_transfer_switch_registers.csv`
  - Collected TX transfer switch register map notes from field documentation/screenshots

## Current scanner defaults

- Port: `/dev/tty.usbserial-FT4WX7ID`
- Slave: `240`
- Start register: `500`
- Count: `300`
- Unlock register: `100`
- Unlock value: `7123`
- Verify register: `101`

## Example usage

```bash
python3 scripts/modbus_register_scanner.py --interval 2 --watch --csv scan.csv
python3 scripts/read_evolution_datetime.py
```
