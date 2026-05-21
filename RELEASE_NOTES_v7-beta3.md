# M5DialBMS v7-beta3 — Release Notes

## What's New

### Web Installer
Flash directly from your browser — no tools, no file downloads:

**https://robertwa1974.github.io/M5Dial-BMS**

Connect your M5Dial via USB-C, click Install, wait ~60 seconds. Done.
Requires Chrome or Edge (Web Serial API). Firefox and Safari are not supported.

If the web installer is unavailable, download `factory.bin` from the Assets below and flash manually at address `0x0` using [esptool-js](https://espressif.github.io/esptool-js/).

### GVRET / SavvyCAN Integration
A GVRET-compatible TCP server runs on port 23 whenever WiFi is active. Connect SavvyCAN to `192.168.4.1:23` (device type: `GVRET (ESP32)`) to see all live CAN traffic — both received frames from CMU modules and transmitted SimpBMS frames.

### WiFi Credentials Updated
Default SSID and password changed to `M5DialBMS` / `M5DialBMS`.

### Build System Fixed
M5GFX 0.2.6 and M5Unified 0.2.5 are now vendored in `lib/` — eliminates the LVGL collision that caused build failures on fresh installs. `espressif32 @ 6.10.0` platform is pinned. All library versions are fixed.

---

## After Flashing

1. Connect to WiFi `M5DialBMS` (password `M5DialBMS`)
2. Open `http://192.168.4.1` in your browser
3. Go to **Settings** and configure your pack (CMU type, modules, cells, thresholds)
4. Connect your CMU modules — data appears on display and dashboard within seconds

> WiFi is **off by default** at boot. Enable via Settings (`WIFI=1`) or long-press the encoder button.

---

## Upgrading from v7-beta2

EEPROM version has been bumped `0x18` → `0x19`. On first boot after flashing, all settings will reset to defaults. Re-enter your CMU type and pack configuration via the web dashboard.

---

## Assets

| File | Description |
|---|---|
| `factory.bin` | Merged binary — bootloader + partitions + firmware. Flash at `0x0`. |

---

*Full changelog: [CHANGELOG.md](CHANGELOG.md) · Full manual: [Operating Manual v7](docs/M5DialBMS_OperatingManual_v7.md)*
