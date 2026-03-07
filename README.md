# TeslaBMS M5Dial v6

ESP32-S3 battery management system display/controller for Tesla Model S/X battery modules and BMW i3 CSC modules, running on the M5Dial (240×240 round display, rotary encoder).

## v6 New Features (over v5.2)

| Feature | Detail |
|---------|--------|
| **CAN RX** | FreeRTOS task on core 0 receives all inbound CAN frames |
| **CAN charger inhibit** | Balancing pauses if charger heartbeat silent >5s on CAN bus |
| **BMW i3 CAN slaves** | Full support for BMW i3 CSC modules (8 modules × 12 cells) |
| **CMU type selection** | Tesla UART or BMW i3 CAN selectable in settings + web UI |
| **WiFi long-press** | Hold encoder button >1s to toggle WiFi on/off at runtime |
| **Larger cell fonts** | Module detail pages use montserrat_20 for cell voltages |
| **Delta symbol fix** | Pack page uses `dV:` instead of broken Unicode Δ glyph |

## Hardware

| Function | GPIO | Connector |
|----------|------|-----------|
| CMU UART RX | 13 | Grove A Yellow |
| CMU UART TX | 15 | Grove A White |
| CAN TX | 1 | Grove B Yellow |
| CAN RX | 2 | Grove B White |
| Drive inhibit | 38 | INPUT_PULLDOWN |
| Encoder A/B/Btn | 40/41/42 | Built-in |
| Power latch | 46 | Built-in |

## EEPROM Version

`0x17` — fresh flash will initialise defaults. Existing v5.x EEPROM will be detected as mismatch and reset to defaults.

## CMU Types

### Tesla UART (default)
- BQ76PL536A daisy-chain protocol at 612,500 baud on Grove A
- Supports up to 20 modules, 4/5/6 cells each (per-module override in settings)
- Full balancing via register writes

### BMW i3 CAN
- 8 CSC modules broadcast autonomously on CAN bus once woken
- Master sends wake frame `0x130` at startup
- Cell voltage frames: `0x3D1`–`0x3D8` (3 sub-frames per module, 4 cells each)
- Temperature frames: `0x3B1`–`0x3B8`
- 12 cells per module, no active balance commands (i3 modules balance internally)
- UART not used in this mode

## CAN Inhibit

When `canInhibitEnabled = 1`, balancing is paused if no frame matching `chargerHeartbeatID` is received for 5 seconds (default ID `0x305` = CAB300 current sensor). GPIO38 inhibit always remains active in parallel.

## Web Dashboard

`http://192.168.4.1` — three tabs:
- **Pack & Modules** — live pack voltage, SoC arc, per-module cell bars with montserrat_20 values
- **CAN Feed** — rolling log of last 50 frames with decoded labels
- **Settings** — all thresholds, CMU type, CAN inhibit, per-module cell count overrides

## Serial Console Commands (v6 additions)

| Command | Example | Description |
|---------|---------|-------------|
| `CMUTYPE=n` | `CMUTYPE=1` | 0=Tesla, 1=BMW i3 (reboot to take effect) |
| `CANINHIBIT=n` | `CANINHIBIT=1` | 0=off, 1=pause bal when charger silent |
| `CHGID=0xHHH` | `CHGID=0x305` | CAN ID to watch for charger heartbeat |

## Libraries

- `m5stack/M5Unified ^0.2.4`
- `m5stack/M5GFX ^0.2.4`
- `lvgl/lvgl ^8.4.0`
- `esphome/AsyncTCP-esphome ^2.1.4`
- `esphome/ESPAsyncWebServer-esphome ^3.3.0`
- `bblanchon/ArduinoJson ^7.3.1`
