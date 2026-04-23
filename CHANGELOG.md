# M5DialBMS — Changelog

---

## v7-beta2 — April 2026

### New Features
- **5 CMU types** supported (up from 2):
  - `0` Tesla UART (BQ76PL536A) — unchanged
  - `1` BMW i3 CSC standard — autonomous broadcast with 0x0A0 enrollment
  - `2` BMW i3 CSC bus pack (Vicinity Motor Corp / BMWI3BUS) — active keepalive TX required
  - `3` BMW Mini-E CSC — alternate voltage encoding (`lo + (hi & 0x3F) × 256`)
  - `4` BMW PHEV SP06/SP41 — polled master/slave, 16 cells/module, 6 sub-frames per poll
- **16-cell module support** — `BMSModule.cellVoltage[]` extended from 12 to 16 entries
- **`getAllVoltTempFromPHEV()`** implemented — reads `phevData[]` from CANManager, populates BMSModule objects for PHEV variant
- **BMWI3BUS active keepalive** — TX burst of 8 frames (0x080–0x087) every 24ms; passive listening alone does not keep bus-pack modules awake
- **Bus-off recovery** — `CANManager::rxTaskFn()` calls `twai_initiate_recovery()` on TWAI_STATE_BUS_OFF detection
- **1-second TX startup delay** — prevents bus-off on cold start before bus is ready

### Bug Fixes
- **CAN TX/RX pins corrected** — `PIN_CAN_TX=2` (GPIO2/yellow), `PIN_CAN_RX=1` (GPIO1/white) to match zombieverter M5Dial pinout; previous assignment was swapped, silently breaking all RX
- **FastLED / TWAI RMT conflict resolved** — `FastLED.show()` on TWAI RX GPIO caused watchdog resets on T-CAN485; LED disabled at startup until GPIO conflicts resolved
- **SN65HVD231 S-pin / slope resistor** — RS pin on Waveshare SN65HVD230 board requires direct GND tie for 500kbps operation; slope-control resistor silently disables RX
- **Buffer overrun fixed** — `BMW_I3_MAX_MODS` corrected from 16→8, `I3_MAX_MODS` from 17→9
- **Web UI CMU type not saving** — `renderSettingsForm()` set select values before DOM injection; moved assignments to after `innerHTML=html`
- **Web UI CMU type options** — select previously showed only 2 options (Tesla/i3); now shows all 5 types with validation accepting 0–4
- **Hardcoded cell loop fixed** — `/api/data` handler `for (int c = 0; c < 6; c++)` replaced with dynamic `nc = bms.getModuleCells(i)` — cells 7–12 were never sent for i3 modules
- **`getModuleCells()` ceiling** — raised from `<= 6` to `<= 16`; now returns correct hardware-fixed count for all BMW CAN variants via switch statement
- **`NUMCELLS` serial command** — validation range raised from `1–6` to `1–16`
- **Web UI `numCells` field** — max attribute raised from 6 to 16; POST validation updated to match
- **Per-module cell override** — web UI select now includes 12S and 16S options; POST handler accepts values 4–16

### Protocol Corrections (verified against reference implementations)
- **Standard i3** — 0x0A0 management bus ID assignment frame sequence corrected (was using placeholder values)
- **Cell voltage byte order** — corrected sub-frame counter position and big-endian/little-endian byte order per variant
- **Mini-E voltage formula** — `lo + (hi & 0x3F) × 256` (differs from standard i3)
- **PHEV poll cycle** — 50ms master poll, 16 cells LE 14-bit encoding confirmed against Tom-evnut/BMWPhevBMS reference

### EEPROM
- Version bumped `0x17` → `0x18` to force settings reset on upgrade from v6

---

## v7-beta1 — March 2026

### New Features
- **BMW i3 CSC bus pack (CMUTYPE=2)** added — initial implementation
- **BMW Mini-E CSC (CMUTYPE=3)** added — initial implementation  
- **BMW PHEV SP06/SP41 (CMUTYPE=4)** added — initial implementation (stub only; `getAllVoltTempFromPHEV()` not yet implemented)
- **Web dashboard** — full 3-tab HTML/CSS/JS dashboard served from PROGMEM
  - Pack & Modules tab with per-cell voltage bars
  - CAN Feed tab with rolling 50-frame log and manual TX tool
  - Settings tab with all configurable parameters
- **Per-module cell count override** — `settings.moduleCells[addr]` array allows individual module slots to override global `numCells` default
- **WiFi runtime toggle** — long encoder press enables/disables WiFi without reboot

### Bug Fixes
- **Duplicate `setup()`/`loop()` linker error** — `BMWPhevBMS.ino` in project root alongside `main.cpp` caused duplicate symbols; moved to `reference/` subfolder and excluded via `build_src_filter`
- **CUV false positives** — `IGNORECELL` threshold (default 2.50V) excludes unpopulated cell inputs from fault detection and min/delta calculations

---

## v6 — February 2026

### New Features
- **BMW i3 CSC standard (CMUTYPE=1)** — first BMW CAN support
  - Wake frame 0x130 at boot
  - Cell frames 0x3D1–0x3D8, temp frames 0x3B1–0x3B8
  - 12 cells per module
- **CAN balance inhibit** — `CANINHIBIT=1` pauses balancing when charger CAN heartbeat (0x305) absent for >5 seconds
- **Charger heartbeat ID configurable** — `CHGID=0xXXX` / `chargerHeartbeatID` setting
- **Per-module cell count** — global default + per-module override stored in EEPROM
- **SoC calibration** — `SOCLO` / `SOCHI` voltage-per-module calibration points

### Breaking Changes
- EEPROM version bumped `0x16` → `0x17`; settings reset on first boot after upgrade

---

## v5 — December 2025

### New Features
- **FreeRTOS CAN RX task** — dedicated task on core 0 for TWAI frame reception
- **Balance inhibit via GPIO38** — drive signal input, active HIGH
- **Auto-balance scheduling** — `AUTOBAL` enable/disable with `BALVOLT`/`BALHYST` thresholds
- **SimpBMS CAN TX** — frames 0x351/0x355/0x356/0x35A/0x35E/0x35F at 1Hz
- **WiFi access point** — basic JSON API for pack data

### Breaking Changes
- EEPROM version bumped to `0x16`

---

## v4 and earlier — Tesla UART only

Initial port from collin80/TeslaBMS to ESP32-S3 / M5Dial hardware. Tesla BQ76PL536A UART support, LVGL display, rotary encoder navigation. Based on ronaegis ESP32-S3 port and bookofthefuture ESP32 fork.
