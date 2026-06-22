# M5DialBMS v7-beta7 — Release Notes

## What's New

### M5Dial 12-Cell Screen Overlap Fix
Alternates display between cells 1-6 and cells 7-12 every 1 second when viewing a module page with 12 cells, utilizing the spacious 6-cell layout to prevent text and progress bar overlap on the round screen.

---

## After Flashing

1. Connect to WiFi `M5DialBMS` (password `M5DialBMS`)
2. Open `http://192.168.4.1` in your browser
3. Go to **Settings** and configure your pack (CMU type, modules, cells, thresholds)
4. Connect your CMU modules — data appears on display and dashboard within seconds

> WiFi is **off by default** at boot. Enable via Settings (`WIFI=1`) or long-press the encoder button.

---

## Upgrading from v7-beta6

Settings will remain preserved in EEPROM. Re-enter your configuration if defaults reset.

---

## Assets

| File | Description |
|---|---|
| `factory.bin` | Merged binary — bootloader + partitions + firmware. Flash at `0x0`. |

---

*Full changelog: [CHANGELOG.md](CHANGELOG.md) · Full manual: [M5DialBMS_OperatingManual_v7.md](docs/M5DialBMS_OperatingManual_v7.md)*
