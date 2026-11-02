# M5DialBMS v7-beta9 (testing) — Release Notes

> ⚠️ **Testing build.** The two new CMU types below are ported from other people's
> work and covered by unit tests, but **neither has been confirmed on hardware yet**.
> Check every reading against a meter before relying on it. Everything from
> v7-beta8 (Tesla, BMW) is unchanged.

## What's New

### VW CMU (MEB / MQB) support — `CMUTYPE=5`
VW battery module CMUs over CAN, ported from Tom de Bree's
[VWBMSV2](https://github.com/Tom-evnut/VW-bms). Thank you, Tom.

- Polls the modules on `0x0BA` every 500 ms (backs off on an empty bus)
- Reads 8, 12 or 13 cell voltages per module, up to 16 modules
- Reads module temperatures (real pack min/max on 0x373) and MEB balance status
- Balances modules 1–12 when auto-balance is on, using `BALVOLT` / `BALHYST`.
  Cells stay live while bleeding, and overvoltage is checked on un-bled readings

### Chrysler Pacifica PHEV BPCM — `CMUTYPE=6`
Reads the Pacifica pack's own BMS (the BPCM), using the frame map from Slyd's bench
DBC for a 2023 pack ([diyelectriccar.com thread](https://www.diyelectriccar.com/threads/tesla-motor-chrysler-pacifica-bpcm-in-smart-fortwo-ed.211649/)). Thank you, Slyd.

- Read only: the Dial sends nothing to the BPCM, which balances its cells itself
- 6 modules × 16 cells, 2 temperatures per module, coolant temperatures, HVIL status
- **Wake:** one 12 V pulse on BPCM pin 9 wakes it, and **two pulses put it to sleep**.
  GPIO13 drives an **external** 12 V high-side switch (never wire GPIO13 to 12 V).
  The Dial pulses once after CAN starts, and again only when you type `W`

### Bug fix
- SavvyCAN/GVRET now shows extended-ID frames as extended

---

## Using VW CMU mode

1. Set `CMUTYPE=5` (serial) or **CMU Type → VW CMU (MEB/MQB)** (web UI). The unit reboots.
2. Wire the CMU logic connector to the CAN transceiver on Grove B (Operating Manual, section 2.6).
3. SoC uses volts per cell (default 3.00/4.20). The cell count comes from the cell frames each module sends.
4. Leave `AUTOBAL=0` until the readings have been checked. Follow the first-session
   bench procedure in the Operating Manual, section 4.5.

## Using Pacifica mode

1. Set `CMUTYPE=6`. The unit reboots.
2. Wire BPCM CAN-H/CAN-L (pins 3/4) to the transceiver on Grove B, +12 V and GND to pins 1/2,
   and jumper the HVIL loop (pins 5–6).
3. Wire GPIO13 (Grove A yellow) to an external high-side switch feeding 12 V to pin 9.
4. After boot, check the six modules appear. If not, type `W` **once** and wait.

See the Operating Manual, section 3.7.

## Open questions (bench testing)

**VW**
- Which cell-ID block MEB modules transmit on, and whether they need the `0x0BA` poll at all
- Cells per MEB module (12s vs 8s modules) versus the reference's 13/12 split; a per-module cell override (8/12/13) is available
- How far a bleeding cell reads low (sets `VW_BLEED_OV_MARGIN`, currently 0.05 V)
- Whether the type 1 or type 2 temperature frames (or both) apply to MEB
- Termination and 12 V / 5 V supply requirements of the module connector

**Pacifica**
- Whether the map holds for packs other than 2023 (2017–2018 packs are a different part family)
- Wake pulse length and polarity, and whether the BPCM stays awake without keep-alive frames
- Whether it really balances standalone
- The undecoded BPCM frames count as an external device, so the SimpBMS summary frames are sent onto the BPCM's bus. Is that harmless?

Capture with SavvyCAN on port 23 (WiFi on) to answer these.

---

## Upgrading

The test installer erases the device. Note your settings first. To keep them,
upload only the application (`pio run -t upload`).

---

## Assets

| File | Description |
|---|---|
| `factory.bin` | Merged binary: bootloader, partitions and firmware. Flash at `0x0`. |

---

*Full changelog: [CHANGELOG.md](CHANGELOG.md) · Full manual: [M5DialBMS_OperatingManual_v7.md](M5DialBMS_OperatingManual_v7.md)*
