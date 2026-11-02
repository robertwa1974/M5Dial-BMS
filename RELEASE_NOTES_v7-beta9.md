# M5DialBMS v7-beta9 — Release Notes

Adds firmware updates over WiFi from the web dashboard, with automatic rollback.
Nothing else changes from v7-beta8.

## What's New

### Web firmware update
Install new firmware from the dashboard, without USB. Saved settings are kept.

1. Download **`firmware.bin`** (not `factory.bin`) from the
   [web installer page](https://robertwa1974.github.io/M5Dial-BMS/) or a release.
2. Turn WiFi on (long-press the encoder), join `M5DialBMS` and open `http://192.168.4.1`.
3. **Settings → Firmware update**: choose `firmware.bin`, enter the WiFi password,
   and click **Upload & restart**.
4. Keep the Dial powered for 30 s after it restarts, so the new version is confirmed.

- The file is checked before anything is written: `factory.bin`, a file for another
  chip, or a wrong password are refused, and the Dial keeps running.
- **Automatic rollback:** if new firmware crashes or resets in its first 30 s, the Dial
  goes back to the previous version and reports it.
- Balancing pauses during the upload. CAN output stops for about 10 s while the Dial
  restarts: **do not update while driving or charging.**

### WiFi password
The update password is the WiFi password. The default `M5DialBMS` is public, so change
it on the serial console: `WIFIPASS=yourpassword` (8–31 characters, no spaces). It can
only be changed over USB.

### Firmware version
The version is shown on the console, the display's settings page and the web page.

---

## Upgrading

**This version has to be installed once over USB.** Earlier versions cannot receive
web updates. After that, later versions can be installed from the dashboard.

- **Web installer** (https://robertwa1974.github.io/M5Dial-BMS/) or `factory.bin`:
  these erase saved settings. Note your settings first, including *Cells per module*
  and the per-module overrides.
- **To keep your settings over USB:** `pio run -t upload` from source.

---

## Assets

| File | Description |
|---|---|
| `factory.bin` | Merged binary: bootloader, partitions and firmware. Flash at `0x0` over USB. |
| `firmware.bin` | Application only, for web updates from the dashboard. |

---

*Full changelog: [CHANGELOG.md](CHANGELOG.md) · Manual section 11.1a: [M5DialBMS_OperatingManual_v7.md](M5DialBMS_OperatingManual_v7.md)*
