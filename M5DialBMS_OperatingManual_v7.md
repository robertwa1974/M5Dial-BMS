# M5DialBMS — Operating Manual v7

**ESP32-S3 BMS Monitor for Tesla & BMW Battery Modules**

Ported from: collin80/TeslaBMS · Tom-evnut/BMWPhevBMS · ronaegis/tesla-bms-esp32s3 · bookofthefuture/BMWPhevBMS-ESP32

---

## Table of Contents

1. [Overview](#1-overview)
2. [Hardware](#2-hardware)
3. [CMU Mode Selection](#3-cmu-mode-selection)
4. [BMW CAN Module Configuration & CAN ID Assignment](#4-bmw-can-module-configuration--can-id-assignment)
5. [Display Interface](#5-display-interface)
6. [WiFi Dashboard](#6-wifi-dashboard)
7. [CAN Bus](#7-can-bus)
8. [Active Balancing](#8-active-balancing)
9. [Serial Console](#9-serial-console)
10. [Fault Codes](#10-fault-codes)
11. [Building and Flashing](#11-building-and-flashing)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Overview

M5DialBMS is a battery management monitor and controller running on the M5Stack M5Dial (ESP32-S3). It supports five CMU types across two physical protocols (UART and CAN), and communicates over a 240×240 round display with a rotary encoder for navigation.

| Feature | Detail |
|---|---|
| CMU types | 5 — Tesla UART, BMW i3 standard, BMW i3 Bus Pack, BMW Mini-E, BMW PHEV SP06/SP41 |
| Max modules | 20 Tesla · 8 BMW i3/Mini-E/Bus Pack · 6 BMW PHEV |
| Cells per module | 4–6 Tesla · 12 BMW i3/Mini-E/Bus Pack · 16 BMW PHEV |
| Display | 240×240 GC9A01 round, LVGL 8.x |
| CAN | SimpBMS-compatible TX + RX (Grove B + transceiver) |
| WiFi | Access Point mode, dashboard at 192.168.4.1 |
| Active balancing | Tesla UART mode only; inhibited in drive mode |
| EEPROM version | 0x18 (v7) — auto-resets to defaults on mismatch |

---

## 2. Hardware

### 2.1 Required Hardware

- M5Stack M5Dial (ESP32-S3FN8, 240×240 GC9A01 round display)
- CMU module(s) — Tesla BQ76PL536A **or** any BMW CAN variant
- CAN transceiver for BMW modes or CAN output — SN65HVD230 recommended
  - **Important:** Use a bare SN65HVD230 transceiver with RS pin tied directly to GND. Waveshare board variants with slope-control resistors on the RS pin can fail at 500 kbps.
  - Grove 4-pin cable(s)

### 2.2 GPIO Pin Assignment

| Function | GPIO | Connector | Notes |
|---|---|---|---|
| CMU UART RX | 13 | Grove A Yellow | Tesla mode only |
| CMU UART TX | 15 | Grove A White | Tesla mode only |
| CAN TX (TWAI) | 2 | Grove B Yellow | Requires CAN transceiver |
| CAN RX (TWAI) | 1 | Grove B White | Requires CAN transceiver |
| Drive inhibit | 38 | Expansion pad | INPUT_PULLDOWN; HIGH = inhibit balancing |
| Encoder A/B/Btn | 40/41/42 | Built-in | Rotary navigation |
| Power latch | 46 | Built-in | Set HIGH immediately on boot |

> **Note on Grove B wiring:** CAN TX is on the **yellow** wire (GPIO2) and CAN RX is on the **white** wire (GPIO1). This matches the zombieverter M5Dial project pinout. Earlier documentation had these reversed — if you have an older wiring harness, swap yellow and white at the transceiver end.

### 2.3 Tesla CMU Wiring — Grove Port A

| CMU Pin | Wire | Grove A Pin | Direction |
|---|---|---|---|
| Pin 5 (5V) | Red | Pin 2 (5V) | Power to CMU |
| Pin 3 (GND) | Green | Pin 1 (GND) | Common ground |
| Pin 4 (TX) | Yellow | Pin 3 (GPIO13) | CMU TX → M5Dial RX |
| Pin 2 (RX) | Blue | Pin 4 (GPIO15) | M5Dial TX → CMU RX |

> **Note:** Tesla CMU modules run at 5V logic. The BQ76PL536A signals are technically 5V but the ESP32-S3 GPIO ESD diodes tolerate this in practice. For permanent installations a BSS138-based level shifter on the TX/RX lines is recommended.

Multiple modules daisy-chain via their second connector. Address assignment is automatic on boot.

### 2.4 CAN Transceiver Wiring — Grove Port B

Required for CAN output (SimpBMS frames) and all BMW CAN modes.

| Grove B Pin | Wire | Transceiver Pin | Notes |
|---|---|---|---|
| Pin 1 (GND) | Black | GND | Also connect to BMW module GND |
| Pin 2 (5V) | Red | VCC | 3.3V transceivers also work |
| Pin 3 (GPIO2) | Yellow | TXD / CRXD | CAN TX |
| Pin 4 (GPIO1) | White | RXD / CTXD | CAN RX |

The CAN transceiver also connects to the BMW module's CAN-H and CAN-L bus wires via its screw terminals.

---

## 3. CMU Mode Selection

v7 supports five CMU types. The mode is stored in flash and survives reboots. Change via the web dashboard Settings tab (`CMU Type` dropdown) or via the serial console command `CMUTYPE=n`. A reboot is required after changing CMU type.

| CMUTYPE | Name | Protocol | Cells/Mod | Max Mods | Notes |
|---|---|---|---|---|---|
| 0 | Tesla UART | BQ76PL536A UART | 4–6 | 20 | Default |
| 1 | BMW i3 CSC standard | CAN 500kbps | 12 | 8 | Autonomous broadcast after enrollment |
| 2 | BMW i3 CSC bus pack | CAN 500kbps | 12 | 8 | Active keepalive required (BMWI3BUS) |
| 3 | BMW Mini-E CSC | CAN 500kbps | 12 | 8 | Different voltage encoding |
| 4 | BMW PHEV SP06/SP41 | CAN 500kbps | 16 | 6 | Polled protocol |

### 3.1 Tesla UART Mode (CMUTYPE=0)

- BQ76PL536A daisy-chain protocol at 612,500 baud on Grove A
- Supports up to 20 modules, 4/5/6 cells each (global or per-module setting)
- Active cell balancing via register writes (BAL_CTRL, BAL_TIME)
- **Boot sequence:** broadcasts wake → broadcasts reset → assigns addresses 1–N sequentially → scans all addresses to confirm presence

### 3.2 BMW i3 CSC Standard (CMUTYPE=1)

- Modules must be enrolled before they broadcast autonomously
- The BMS sends management bus ID assignment frames on 0x0A0
- Once enrolled, each module broadcasts cell voltage frames on 0x3D1+n and temperature frames on 0x3B1+n
- Grove A (UART) is not used in this mode

### 3.3 BMW i3 CSC Bus Pack / Vicinity Motor Corp (CMUTYPE=2)

- Used with BMW i3 battery packs sourced from Vicinity Motor Corp buses and similar
- Modules do **not** respond to 0x0A0 enrollment — they have fixed slot addresses
- The BMS must send active keepalive bursts on 0x080–0x087 every 24ms or modules go silent
- Cell data arrives autonomously on 0x12n–0x15n once keepalives are running

### 3.4 BMW Mini-E CSC (CMUTYPE=3)

- Similar to bus pack but uses a different cell voltage encoding formula
- Keepalive frames sent on 0x088–0x08B

### 3.5 BMW PHEV SP06/SP41 (CMUTYPE=4)

- Polled master/slave protocol — modules are completely silent until polled
- BMS sends a poll command to each slot every 50ms (rotating through slots 0–5)
- 16 cells per module across 6 sub-frames per poll cycle
- Little-endian 14-bit voltage encoding, 1mV/bit

---

## 4. BMW CAN Module Configuration & CAN ID Assignment

This section covers how to discover, address, and verify BMW CAN modules for each supported variant.

### 4.1 Standard i3 CSC Enrollment (CMUTYPE=1)

Standard i3 modules power up in an unenrolled state and must be assigned a management bus ID before they will broadcast cell data.

**Step 1 — Find unassigned modules**

Send the serial command:
```
I3FIND
```
or press `E` in the console. The BMS transmits a find-unassigned frame (`0x0A0`, data `[37 FF FF FF FF FF FF FF]`). Any unenrolled module responds on `0x4A0` with its 8-byte DMC hardware ID. The console prints:

```
i3 unassigned CSC found: DMC=[XX XX XX XX XX XX XX XX]
```

**Step 2 — Assign a slot**

Send:
```
I3ASSIGN
```
or press `A`. The BMS runs the 4-frame assignment sequence using the DMC bytes from the discovery response. On success the console prints:

```
i3 CSC assigned to slot N
```

Repeat `I3FIND` + `I3ASSIGN` for each additional module. Slots are assigned sequentially (1, 2, 3…). Once assigned, each module broadcasts autonomously and the display updates within 1–2 seconds.

**Step 3 — Verify**

The Pack Summary page will show the module count incrementing. Individual cell voltages appear on the module detail pages.

> **Note:** Assignment is volatile in the CSC module — it is lost on power cycle. The BMS re-runs enrollment automatically at boot for CMUTYPE=1. Ensure all modules are powered before booting the BMS.

### 4.2 Bus Pack / Vicinity (CMUTYPE=2)

No enrollment procedure is required. Module slot addresses are fixed by the physical harness position:

| Slot | Cell frame ID | Temp frame ID |
|---|---|---|
| 0 | 0x120–0x150 | 0x170 |
| 1 | 0x121–0x151 | 0x171 |
| … | … | … |
| 7 | 0x127–0x157 | 0x177 |

The BMS begins sending keepalive bursts immediately at boot. Modules appear as `existing` the first time a valid cell frame arrives. No console commands are needed.

**Wiring note:** CAN-H and CAN-L polarity matters. If modules do not appear within 5 seconds of boot with keepalives running, swap CAN-H and CAN-L at the transceiver screw terminals.

### 4.3 PHEV SP06/SP41 (CMUTYPE=4)

No enrollment. The BMS polls slots 0–5 in rotation at 50ms intervals. A module is marked `existing` on first valid poll response. Slots that never respond remain absent and are not counted.

The poll command ID and response ID vary by sub-frame; this is handled automatically by the firmware. No user configuration is required beyond setting `CMUTYPE=4`.

### 4.4 Verifying CAN Activity

The **CAN Feed** tab in the web dashboard shows a rolling log of the last 50 frames with decoded labels. Use this to confirm:

- Keepalive frames are being sent (0x080–0x087 for BMWI3BUS)
- Cell voltage frames are arriving (0x120–0x15F for bus pack, 0x3D1–0x3D8 for standard i3)
- Temperature frames are arriving (0x170–0x17F or 0x3B1–0x3B8)

The serial console also logs RX activity at log level 0 (DEBUG):
```
LOGLEVEL=0
```

---

## 5. Display Interface

### 5.1 Pack Summary Page (Page 0)

Shown at boot. Short-press the encoder to return here from any page.

- **Pack voltage** — large, white. Turns red if any module is faulted
- **State of Charge %** — blue, linear interpolation between SoC Lo and SoC Hi voltage per module
- **Lowest / highest cell voltage** across all modules
- **Cell delta (dV)** — highest minus lowest. Turns amber if delta > 50mV
- **Average pack temperature**
- **Module count** — number of responding modules
- **WiFi IP address** (or `WiFi: off`)
- **Arc colour** — green = healthy, red = any fault active

### 5.2 Module Detail Pages

Rotate the encoder to navigate through detected modules. Each page shows:

- Module number and total module voltage
- All cell voltages with colour-coded bar graphs
  - Green = normal
  - Red = below UV threshold
  - Amber = above OV threshold
- T1 and T2 temperature readings
- Fault badge: `OK` (green) or `FAULT:0xXX` (red)

Cell count displayed adapts automatically to the CMU type: 6 cells for Tesla, 12 for i3/Mini-E/Bus Pack, 16 for PHEV.

### 5.3 Settings Page

Long-press the encoder button (≥ 1 second) to toggle WiFi on/off. The Settings page is accessible via the web dashboard only (not on the display in v7).

### 5.4 Encoder Controls

| Action | Result |
|---|---|
| Rotate clockwise | Next module detail page |
| Rotate counter-clockwise | Previous module detail page |
| Short press (< 1 s) | Return to pack summary (page 0) |
| Long press (≥ 1 s) | Toggle WiFi on/off at runtime |

---

## 6. WiFi Dashboard

### 6.1 Access

WiFi operates in Access Point mode.

- **SSID:** `TeslaBMS`
- **Password:** `teslapack`
- **URL:** `http://192.168.4.1`

WiFi can be enabled at boot (Settings tab or `WIFI=1` serial command) or toggled at runtime with a long encoder press.

### 6.2 Dashboard Tabs

**Pack & Modules**
Live pack voltage, SoC, cell lo/hi/delta, temperature. Per-module cards with individual cell voltage bars scaled to the correct cell count for the active CMU type.

**CAN Feed**
Rolling log of the last 50 CAN frames with decoded labels. Includes a manual CAN frame transmit tool for diagnostics.

**Settings**
All thresholds, CMU type selection, CAN inhibit mode, charger heartbeat ID, per-module cell count overrides. All changes are saved instantly to flash.

### 6.3 Settings Reference (Web UI)

| Field | Description |
|---|---|
| CMU Type | Selects module protocol (reboot required) |
| Cells per module | Global default: 4–16 |
| Modules in series | Used for SoC voltage calculation |
| Modules in parallel | Used for pack voltage display |
| SoC 0% / 100% V/module | Linear SoC interpolation endpoints |
| OV limit | Overvoltage trip threshold per cell |
| UV limit | Undervoltage trip threshold per cell |
| Ignore cell below | Cells below this voltage are excluded from min/delta (use 2.5V for unpopulated inputs) |
| Over/Under temp | Temperature trip thresholds |
| Balance target / hyst | Balancing voltage target and dead-band (Tesla only) |
| Auto-balance | Enable/disable automatic balancing |
| WiFi at boot | Persist WiFi state across reboots |
| Balance Inhibit | `GPIO only` or `GPIO + CAN charger heartbeat` |
| Charger HB ID | CAN ID to watch for charger presence (default 0x305) |
| Per-module cell count | Override global default for individual module slots (0 = inherit global) |

---

## 7. CAN Bus

### 7.1 Outbound SimpBMS Frames (TX)

Transmitted every 1 second at 500 kbps. Compatible with Victron, SMA, and other SimpBMS-compatible inverters/chargers.

| CAN ID | Content | Encoding |
|---|---|---|
| 0x351 | Charge voltage limit · charge current limit · discharge current limit · discharge voltage limit | 0.1V/A per bit, little-endian signed 16-bit |
| 0x355 | State of Charge % · State of Health (fixed 100%) | 1%/bit, little-endian |
| 0x356 | Pack voltage · current (CAB300) · average temperature | 0.01V/bit · 0.1A/bit · 0.1°C/bit, signed |
| 0x35A | Alarm and fault bitmask (OV, UV, OT flags) | Bitmask |
| 0x35E | Manufacturer string `TeslaBMS` | ASCII |
| 0x35F | Chemistry `LI` · hardware v1.0 · firmware v7.0 | ASCII/BCD |

### 7.2 Inbound CAN (RX)

A dedicated FreeRTOS task on core 0 receives all frames. Received frames serve two purposes:

- **Charger heartbeat detection** — any frame matching the configured Charger HB ID (default 0x305) resets a 5-second timeout. Current from frame 0x305/0x306 is forwarded to the 0x356 TX frame.
- **BMW CAN cell/temp decoding** — frames from all BMW variants are decoded and stored in per-module data structures, then consumed by `getAllVoltTempFromCAN()` or `getAllVoltTempFromPHEV()` each main loop cycle.

### 7.3 BMW CAN Frame Map

**Standard i3 (CMUTYPE=1)**

| Frame ID | Content |
|---|---|
| 0x3D1–0x3D8 | Cell voltages, modules 1–8 (3 sub-frames × 4 cells, 1mV/bit big-endian) |
| 0x3B1–0x3B8 | Temperatures, modules 1–8 (2 sensors, 0.1°C/bit signed) |
| 0x0A0 | Management bus command (TX — enrollment) |
| 0x4A0 | Unassigned CSC reply (RX — enrollment) |

**Bus Pack / Mini-E (CMUTYPE=2 and 3)**

| Frame ID | Content |
|---|---|
| 0x080–0x087 | Keepalive TX (8 frames per burst, every 24ms) |
| 0x120–0x15F | Cell voltages, slots 0–7 (multiple sub-frames, LE 16-bit 1mV/bit) |
| 0x170–0x17F | Temperatures, slots 0–7 (data[4]–40 = °C) |

**PHEV SP06/SP41 (CMUTYPE=4)**

| Frame ID | Content |
|---|---|
| Poll command | Sent by BMS every 50ms per slot | 
| Response frames | 6 sub-frames per module, 16 cells LE 14-bit 1mV/bit |

### 7.4 Balance Inhibit via CAN

When CAN Balance Inhibit is enabled (`CANINHIBIT=1`), balancing pauses if no charger heartbeat is received for more than 5 seconds.

| CAN Status | Meaning |
|---|---|
| `CAN=off` | CAN inhibit disabled — GPIO38 only |
| `CAN=CHG-OK` | Charger heartbeat seen recently — balancing allowed |
| `CAN=TIMEOUT` | No heartbeat for >5 s — balancing inhibited |

---

## 8. Active Balancing

Balancing applies to **Tesla UART mode only**. BMW CSC modules manage their own internal balancing.

Balancing is disabled by default. Enable via `AUTOBAL=1` or the web Settings tab.

### 8.1 Balancing Logic

1. Each cycle, the lowest valid cell voltage across the pack is found (ignoring cells below `IGNORECELL`)
2. Any cell above both the pack low voltage **and** the `BALVOLT` threshold has its `BAL_CTRL` bit set
3. A 60-second hardware timer in the BQ76PL536A limits each balance burst
4. Balancing repeats each 1-second main loop cycle while conditions are met

### 8.2 Inhibit Sources

- **GPIO38 HIGH** — vehicle drive signal. Wire to a 3.3V drive-active signal (5V via divider).
- **CAN heartbeat timeout** — if `CANINHIBIT=1` and no charger frame seen for 5 seconds.
- **Pack fault active** — any module reporting a non-CUV fault suspends balancing.

---

## 9. Serial Console

### 9.1 Connection

Connect via USB-C. Open a serial monitor at **115200 baud**, 8-N-1, with line endings enabled (LF or CRLF).

PlatformIO: `pio device monitor`

### 9.2 Short Commands (single key + Enter)

| Key | Action |
|---|---|
| `H` or `?` | Print help menu |
| `S` | Sleep all CMU boards (Tesla only) |
| `W` | Wake all CMU boards (Tesla only) |
| `C` | Clear all faults on all boards |
| `F` | Re-scan for connected boards |
| `R` | Renumber boards from address 1 (Tesla only) |
| `B` | Trigger one cell balancing cycle |
| `E` | Find unassigned BMW i3 standard CSC module |
| `A` | Assign found BMW i3 standard CSC module to next slot |
| `p` | Toggle pack summary printout (3-second interval) |
| `d` | Toggle pack detail printout (3-second interval) |

### 9.3 Configuration Commands (KEY=value + Enter)

All settings are saved to flash automatically after each valid command.

| Command | Range | Description |
|---|---|---|
| `VOLTLIMHI=4.20` | 0.0–6.0 V | Overvoltage threshold per cell |
| `VOLTLIMLO=3.00` | 0.0–6.0 V | Undervoltage threshold per cell |
| `IGNORECELL=2.50` | 0.0–3.0 V | Ignore cells below this voltage (2.5V for unpopulated inputs) |
| `TEMPLIMHI=55.0` | 0–100 °C | Over-temperature threshold |
| `TEMPLIMLO=-10.0` | -40–30 °C | Under-temperature threshold |
| `BALVOLT=4.10` | 0.0–6.0 V | Balance target voltage per cell |
| `BALHYST=0.04` | 0.0–1.0 V | Hysteresis below BALVOLT |
| `AUTOBAL=1` | 0 or 1 | Enable/disable automatic balancing |
| `NUMCELLS=6` | 1–16 | Global default cells per module |
| `NUMSERIES=8` | 1–62 | Modules in series |
| `NUMPARALLEL=1` | 1–10 | Modules in parallel |
| `SOCLO=18.0` | V/module | Voltage at 0% SoC |
| `SOCHI=25.2` | V/module | Voltage at 100% SoC |
| `WIFI=1` | 0 or 1 | Enable WiFi on next boot |
| `LOGLEVEL=1` | 0–4 | 0=Debug 1=Info 2=Warn 3=Error 4=Off |
| `CMUTYPE=0` | 0–4 | CMU type (reboot required after change) |
| `CANINHIBIT=1` | 0 or 1 | Enable CAN charger heartbeat balance inhibit |
| `CHGID=0x305` | 0x001–0x7FF | CAN ID to watch for charger heartbeat |
| `BATTERYID=1` | 1–14 | Battery ID in SimpBMS CAN frames |

---

## 10. Fault Codes

### 10.1 BQ76PL536A Fault Register (Tesla UART)

| Bit | Meaning |
|---|---|
| 0x01 | Cell overvoltage (COV) |
| 0x02 | Cell undervoltage (CUV) — expected on packs with fewer than 6 cells; suppressed in fault display when IGNORECELL is set appropriately |
| 0x04 | CRC error in UART packet |
| 0x08 | Power-on reset |
| 0x10 | Test fault active |
| 0x20 | Internal registers inconsistent |

The on-display fault badge masks bit 0x02 (CUV) by default. The raw register is always visible in the serial console and web dashboard.

### 10.2 BMW CAN Modes

BMW CSC modules do not expose a fault register over CAN. Fault detection is:

- **Cell OV/UV** — any cell reading outside the configured thresholds
- **Module timeout** — no CAN frame received from a module for more than 3 seconds (module is marked absent)
- **PHEV errorWord** — non-zero `errorWord` in poll response sets the module fault flag

---

## 11. Building and Flashing

### 11.1 Requirements

- Visual Studio Code with the PlatformIO IDE extension
- USB-C cable (data-capable)

### 11.2 Steps

1. Open the `TeslaBMS_M5Dial` folder in VS Code.
2. PlatformIO auto-detects `platformio.ini` and downloads the ESP32-S3 toolchain and libraries.
3. Edit `src/bms_config.h` to change `BMS_NUM_SERIES` or `BMS_NUM_PARALLEL` before first flash if needed.
4. Click **Build** (✓) to verify compilation.
5. Click **Upload** (→) to flash. Hold the M5Dial power button if needed to enter download mode.
6. Open the serial monitor at 115200 baud to view boot logs and enter commands.

> **Note:** EEPROM version is 0x18. Upgrading from v6 firmware (EEPROM 0x17) will trigger a one-time reset of all settings to defaults on first boot. Re-enter your settings after the upgrade.

### 11.3 Library Dependencies

| Library | Version |
|---|---|
| m5stack/M5Unified | ^0.2.4 |
| m5stack/M5GFX | ^0.2.4 |
| lvgl/lvgl | ^8.4.0 |
| esphome/AsyncTCP-esphome | ^2.1.4 |
| esphome/ESPAsyncWebServer-esphome | ^3.3.0 |
| bblanchon/ArduinoJson | ^7.3.1 |

---

## 12. Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| No modules found, TX errors `0x106` | Listen-only mode still active | Change `TWAI_MODE_LISTEN_ONLY` to `TWAI_MODE_NORMAL` in `CANManager::begin()` |
| No RX frames, TX errors only | CAN TX/RX pins swapped | Confirm `PIN_CAN_TX=2`, `PIN_CAN_RX=1` in `bms_config.h`; swap yellow/white wires if needed |
| Bus-off state, watchdog resets | SN65HVD230 RS pin floating | Tie RS pin directly to GND on the transceiver board |
| M5Stack CANBus Unit not working | Unit uses UART internally | Replace with bare SN65HVD230 transceiver wired directly to Grove B |
| Modules appear then disappear | CAN bus not terminated | Add 120Ω between CAN-H and CAN-L at each end of the bus |
| Cell values show 0.000V in web UI | `for (int c = 0; c < 6; c++)` hardcoded | Flash updated `WiFiManager.cpp` with dynamic cell loop |
| Web UI CMU type not saving | DOM injection ordering bug | Flash updated `WiFiManager.cpp` with fixed `renderSettingsForm()` |
| EEPROM mismatch on boot | Firmware version change | Normal — defaults are written automatically; re-enter settings |
| FastLED / TWAI watchdog reset | RMT peripheral conflict on CAN RX GPIO | Disable LED calls before TWAI init; confirmed fixed in v7 |
| PHEV modules silent | Poll commands not reaching modules | Confirm `CMUTYPE=4`; check CAN wiring; PHEV modules are completely silent until polled |

---

*TeslaBMS M5Dial — Operating Manual v7 · EEPROM 0x18 · April 2026*
