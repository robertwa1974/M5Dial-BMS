# M5Dial BMS v6

ESP32-S3 battery management system display/controller for Tesla Model S/X battery modules and BMW i3 CSC and PHEV modules, running on the M5Dial (240×240 round display, rotary encoder).

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

### VW CMU — MEB / MQB (CMUTYPE=5)
- VW battery module CMUs on CAN 500 kbit/s; protocol ported from Tom de Bree's
  [VWBMSV2](https://github.com/Tom-evnut/VW-bms) (commit 4a6a5b9)
- Master broadcasts a two-frame poll on `0x0BA` every 500 ms (with empty-bus backoff)
- Cell voltages: standard IDs `0x1B0`–`0x1EE`; the ID encodes module (1–16) and sub-frame
- 8, 12 or 13 cells per module, from the sub-frames each module actually sends (modules 1–9 up to 13, 10–16 up to 12)
- Temperatures: extended IDs `0x1A555401`–`0x1A55543F` (type 1) and `0x1A5555F0`–`0x1A5555FE` (type 2, MEB)
- MEB balance status: extended IDs `0x16A95471`–`0x16A9547F`
- Balancing via extended-ID commands (modules 1–12), same duty cycle as the reference:
  ~5 s measure / ~25 s bleed, gated on `BALVOLT` and `BALHYST`. Cells stay decoded while
  bleeding; selection and overvoltage use un-bled readings (see the Operating Manual, 8.1b)
- **First bench session:** follow the Operating Manual, section 4.5 — capture and decode
  only, check cell mapping against a multimeter, then enable balancing
- **Unconfirmed on MEB hardware** — still to be checked on the bench:
  - which cell-ID block MEB modules transmit on, and whether they need the `0x0BA` poll at all
  - cells per MEB module (12s vs 8s modules) versus the reference's 13/12 split
  - how far a bleeding cell reads low (`VW_BLEED_OV_MARGIN`, 0.05 V placeholder)
  - whether the type 1 or type 2 temperature frames (or both) apply to MEB
  - termination and 12 V / 5 V supply requirements of the module connector

### Chrysler Pacifica PHEV BPCM (CMUTYPE=6)
- The pack's own BMS (BPCM, LG) on CAN 500 kbit/s; frame map from Slyd's bench DBC
  ([diyelectriccar.com thread](https://www.diyelectriccar.com/threads/tesla-motor-chrysler-pacifica-bpcm-in-smart-fortwo-ed.211649/), 2023 pack)
- **RX only**: the Dial sends nothing to the BPCM. It balances internally (reported, not yet verified)
- Cell voltages: `0x487`–`0x49E`, `0x487 + 4*(module-1) + sub`, 4 cells per frame, 16-bit big-endian, 1 mV/bit
- 6 modules × 16 cells, 2 temperatures per module on `0x4A0` (modules 1–3) / `0x4A1` (4–6), byte − 40 °C
- Coolant outlet/inlet on `0x307`; HVIL status on `0x150` (logged when it changes)
- **Wake**: BPCM connector pin 9 needs a 12 V pulse (one pulse wakes, two pulses sleep).
  GPIO13 (Grove A yellow, free in CAN modes) drives an **external** high-side switch for it:
  one 200 ms pulse after CAN starts, and again on the console `W` command. Never repeated
  automatically, so a missed wake doesn't turn into a sleep
- BPCM low-voltage connector (black, 12-pin, per Slyd): 1 = +12 V, 2 = GND, 3 = CAN-H,
  4 = CAN-L, 5–6 = HVIL loop (jumper), 9 = wake
- **Unconfirmed** — still to be checked on the bench:
  - the whole map on packs other than 2023 (2017–2018 packs are a different part family)
  - wake pulse length and polarity, and whether the BPCM stays awake without keep-alive frames
  - whether it really balances standalone
  - the BPCM sends ~44 IDs; the undecoded ones count as an external device and enable
    the SimpBMS summary TX onto the BPCM's bus
  - the 11-bit `0x150` field Slyd labels SOC (×0.192, 0–393: looks more like pack voltage)

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
| `CMUTYPE=n` | `CMUTYPE=1` | 0=Tesla, 1=BMW i3, 2=BMW i3 Bus, 3=BMW Mini-E, 4=BMW PHEV, 5=VW CMU (reboot to take effect) |
| `CANINHIBIT=n` | `CANINHIBIT=1` | 0=off, 1=pause bal when charger silent |
| `CHGID=0xHHH` | `CHGID=0x305` | CAN ID to watch for charger heartbeat |

## Tests

Host-side unit test for the VW CMU decoders. CI runs it on every push and pull
request (`.github/workflows/test.yml`); locally it needs any C++11 host compiler:

```
g++ -std=c++11 -Wall -Wextra -Isrc -Itest/stubs test/vw_decode_test.cpp -o vw_decode_test
./vw_decode_test
```

`test/vw_reference_tables.h` is generated from the VWBMSV2 source by
`tools/gen_vw_reference_tables.py` (see the script header for usage).

## Libraries

- `m5stack/M5Unified ^0.2.4`
- `m5stack/M5GFX ^0.2.4`
- `lvgl/lvgl ^8.4.0`
- `esphome/AsyncTCP-esphome ^2.1.4`
- `esphome/ESPAsyncWebServer-esphome ^3.3.0`
- `bblanchon/ArduinoJson ^7.3.1`
