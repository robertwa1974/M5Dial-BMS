# M5DialBMS v7-beta8 — Release Notes

## What's New

### VW CMU (MEB / MQB) support — `CMUTYPE=5`
VW battery module CMUs over CAN, ported from Tom de Bree's
[VWBMSV2](https://github.com/Tom-evnut/VW-bms) — thank you, Tom.

- Polls the modules on `0x0BA` every 500 ms (backs off on an empty bus)
- Reads 8, 12 or 13 cell voltages per module, up to 16 modules
- Reads module temperatures (real pack min/max on 0x373) and MEB balance status
- Balances modules 1–12 when auto-balance is on, using `BALVOLT` / `BALHYST`;
  cells stay live while bleeding, with overvoltage checks on un-bled readings

> ⚠️ **Experimental.** The protocol is ported from VWBMSV2 and covered by unit
> tests, but has **not yet been confirmed on MEB hardware**. Check cell
> voltages against a meter before enabling balancing. See *Open questions*
> below.

### Bug fixes
- BMW PHEV cells 7–16 no longer read 0.000 V on the display and web UI
  (separate PR — needs a PHEV bench retest)
- BMW PHEV temperature sensors T3/T4 are now stored
- SavvyCAN/GVRET now shows extended-ID frames as extended

---

## Using VW CMU mode

1. Set `CMUTYPE=5` (serial) or **CMU Type → VW CMU (MEB/MQB)** (web UI); the unit reboots
2. Wire the CMU logic connector to the CAN transceiver on Grove B — see the
   Operating Manual, section 2.6
3. Set `NUMSERIES`, `SOCLO` and `SOCHI` for your module voltage
4. Leave `AUTOBAL=0` until the readings have been checked — follow the
   first-session bench procedure in the Operating Manual, section 4.5

## Open questions (bench testing)

- Which cell-ID block MEB modules actually transmit on, and whether they need the `0x0BA` poll at all
- Cells per MEB module (12s vs 8s modules) versus the reference's 13/12 split — per-module cell override (8/12/13) is available
- How far a bleeding cell reads low (sets `VW_BLEED_OV_MARGIN`, currently 0.05 V)
- Whether the type 1 or type 2 temperature frames (or both) apply to MEB
- Termination and 12 V / 5 V supply requirements for the module connector

Capture with SavvyCAN on port 23 (WiFi on) to answer these.

---

## Upgrading from v7-beta7

Settings are preserved (EEPROM version unchanged).

---

## Assets

| File | Description |
|---|---|
| `factory.bin` | Merged binary — bootloader + partitions + firmware. Flash at `0x0`. |

---

*Full changelog: [CHANGELOG.md](CHANGELOG.md) · Full manual: [M5DialBMS_OperatingManual_v7.md](docs/M5DialBMS_OperatingManual_v7.md)*
