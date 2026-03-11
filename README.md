# modbus-tools

Small Modbus utilities.

## Included

- `modbus_register_scanner.py` — read-only Modbus RTU scanner/poller with optional unlock write, progress output, CSV logging, and skip-on-read-failure behavior.

## Current defaults

- Port: `/dev/tty.usbserial-FT4WX7ID`
- Slave: `240`
- Start register: `500`
- Count: `300`
- Unlock register: `100`
- Unlock value: `7123`
- Verify register: `101`

## Usage

```bash
python3 modbus_register_scanner.py --interval 2 --watch --csv scan.csv
```
