# TeslaBMS M5Dial — Release Notes
## v7-beta2 — April 2026

> **Hardware:** M5Stack M5Dial (ESP32-S3)
> **EEPROM version:** 0x17 (unchanged from v6 — settings survive upgrade)
> **CAN pins:** TX = GPIO2 (Grove B yellow) · RX = GPIO1 (Grove B white)

---

### Overview

This release completes multi-CMU support, fixes a root-cause CAN hardware bug that silently broke all RX since the initial ESP32-S3 port, and hardens the TWAI driver against TX overflow crashes adapted from hpeyerl's bmw-i3-bms-tcan485 project.

---

### Breaking Changes

**CAN TX/RX pins are swapped relative to v6 documentation.**

The v6 manual had `PIN_CAN_TX=1` (white) and `PIN_CAN_RX=2` (yellow). The correct assignment — confirmed against the zombieverter M5Dial project and verified on hardware — is:

```
PIN_CAN_TX = 2   (Grove B yellow wire)
PIN_CAN_RX = 1   (Grove B white wire)
```

If you are upgrading from v6 with existing wiring, swap the yellow and white wires at the transceiver end, or swap them in `bms_config.h`.

**No EEPROM reset required.** Version byte is unchanged at `0x17`.

---

### What's New

#### Multi-CMU Support — all 5 types now fully implemented

| CMUTYPE | Name | Status |
|---|---|---|
| 0 | Tesla UART (BQ76PL536A) | Unchanged |
| 1 | BMW i3 CSC standard | Autonomous broadcast, 0x0A0 enrollment |
| 2 | BMW i3 CSC bus pack (BMWI3BUS) | Active 24ms keepalive, fixed slot addressing |
| 3 | BMW Mini-E CSC | Alternate voltage encoding |
| 4 | BMW PHEV SP06/SP41 | Polled 50ms cycle, 16 cells/module |

`getAllVoltTempFromPHEV()` is now fully implemented — previously it was declared in the header and called in `main.cpp` but never defined, causing a linker error on any PHEV build.

#### 12-cell and 16-cell display support

The module detail page previously only supported up to 6 cells regardless of CMU type. The LVGL page now rebuilds automatically when the cell count changes — fixing the symptom where only cells 7–12 appeared on screen for i3 modules. Layout geometry adapts: 3-row for ≤6 cells (Tesla), 6-row compact for 12-cell i3/Mini-E/Bus Pack.

#### Web UI — cells per module dynamic

The `/api/data` JSON handler previously hardcoded `for (int c = 0; c < 6; c++)`. This is now `int nc = bms.getModuleCells(i); for (int c = 0; c < nc; c++)`. Individual cell voltages now appear correctly in the web dashboard for all CMU types.

#### Web UI — CMU type setting now saves correctly

`renderSettingsForm()` was setting select element values before the HTML existed in the DOM — those assignments silently did nothing. The CMU type, balance inhibit, and charger heartbeat ID selects now populate correctly after DOM injection. Selecting a CMU type in the Settings tab and pressing Apply & Save now correctly persists the choice.

#### CAN bus hardening (from hpeyerl/bmw-i3-bms-tcan485)

Five improvements ported from hpeyerl's upstream commit 59bf762. Changes 1–4 apply to **all CMU types** — they fix fundamental TWAI driver behaviour that is independent of the BMW protocol running above it. The BMWI3BUS variant made these issues visible first (its 8-frame burst saturates the default TX queue), but the fixes are correct practice for any ESP32 TWAI application. Change 5 is BMWI3BUS-specific.

1. **TX/RX queue sizes — all variants** — TWAI TX queue raised from the ESP-IDF default of 5 to 8; RX queue raised to 32. The default TX queue of 5 is too small for the 6-frame SimpBMS summary burst, which could already overflow under any CMU type. RX queue of 32 gives adequate headroom when multiple modules broadcast simultaneously. Eliminates `ESP_ERR_TIMEOUT` TX errors under normal load.

2. **Full bus-off recovery — all variants** — `rxTaskFn` now responds to `TWAI_STATE_BUS_OFF` by calling `twai_stop()` + `twai_driver_uninstall()`, waiting 200ms, then reinstalling the driver with the same configuration. This is more reliable than `twai_initiate_recovery()` alone and also resets module `lastSeenMs` so keepalive sequencing restarts cleanly after recovery. Bus-off can occur on any variant from a wiring fault, noise spike, or disconnected termination resistor.

3. **TX state guard in `sendFrame()` — all variants** — checks `TWAI_STATE_RUNNING` before calling `twai_transmit()`. Calling `twai_transmit()` during bus-off with frames pending in the queue causes `tx_msg_count` to go negative inside the ESP-IDF TWAI driver, triggering an assert crash. This guard prevents that crash regardless of CMU type. TX timeout also changed from `pdMS_TO_TICKS(5)` to `0` (non-blocking) — frames are dropped rather than blocking the loop task if the queue is momentarily full.

4. **`externalDeviceSeen` gate — all variants** — SimpBMS summary frames (0x351/355/356/35A/35E/35F) are only transmitted once a non-CSC frame has been received, confirming an external device (inverter, charger, or CAN analyser) is present to ACK them. Without an ACK, every summary frame generates a TX error and contributes to bus-off conditions. This applies regardless of CMU type — a standalone Tesla UART pack with no inverter connected suffers the same problem. The flag latches permanently once set.

5. **Skip `sendI3WakeFrame()` for BMWI3BUS — BMWI3BUS only** — the 0x130 wake frame sequence is specific to the standard i3 CSC enrollment protocol. Bus-pack modules do not use it and do not respond to it. Skipping it for `CMUTYPE=2` eliminates one unnecessary TX frame at boot with no effect on other variants.

---

### Bug Fixes

| # | Component | Description |
|---|---|---|
| 1 | `bms_config.h` | `PIN_CAN_TX`/`PIN_CAN_RX` corrected — root cause of all CAN RX failures since initial ESP32-S3 port |
| 2 | `Display.cpp` | Module page rebuilds when cell count changes — fixes cells 7–12 only appearing for i3 modules |
| 3 | `Display.cpp` | `MAX_CELLS_DISPLAY` correctly used