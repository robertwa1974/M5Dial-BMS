# M5DialBMS v7-beta6 — Release Notes

## What's New

### BMW i3 Bus Checksum Bug Fix
Resolves the issue where battery modules rejected keepalive commands and the screen/web UI displayed 0V. We implemented a correct 9-byte CRC8 checksum calculation covering the 2-byte CAN ID and 7 payload bytes (including the rolling counter).

### Web UI 12-Cell Display Fix
Resolved the limitation where the web UI and JSON API capped display output at 6 cells. The system now dynamically shows 12 cell fields for BMW i3/Mini-E variants, and 16 cell fields for PHEV.

### Auto-Reboot on CMU Type Change
When changing the CMU Type via the Web UI or Serial Console (`CMUTYPE`), the device automatically saves your configuration and triggers an automatic reboot (`ESP.restart()`) after 1 second.

### Mutex-Protected Logging & CAN Throttling
We implemented FreeRTOS mutex synchronization in logging functions to prevent character interleaving, and rate-limited `0x100` heartbeat log prints to avoid serial buffer overflow and packet drops.

---

## After Flashing

1. Connect to WiFi `M5DialBMS` (password `M5DialBMS`)
2. Open `http://192.168.4.1` in your browser
3. Go to **Settings** and configure your pack (CMU type, modules, cells, thresholds)
4. Connect your CMU modules — data appears on display and dashboard within seconds

> WiFi is **off by default** at boot. Enable via Settings (`WIFI=1`) or long-press the encoder button.

---

## Upgrading from v7-beta5

Settings will remain preserved in EEPROM. Re-enter your configuration if defaults reset.

---

## Assets

| File | Description |
|---|---|
| `factory.bin` | Merged binary — bootloader + partitions + firmware. Flash at `0x0`. |

---

*Full changelog: [CHANGELOG.md](CHANGELOG.md) · Full manual: [M5DialBMS_OperatingManual_v7.md](docs/M5DialBMS_OperatingManual_v7.md)*
