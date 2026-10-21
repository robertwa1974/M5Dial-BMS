# Test plan: BMW PHEV battery modules (SP06 / SP41 / SP44)

Thank you for testing. This firmware has **not yet been run on a BMW PHEV pack**. The
fixes were found by comparing the code with Tom de Bree's BMWPhevBMS, and checked with
unit tests, but only real modules can confirm them. Your logs are what we need. A test
that fails and comes with good logs is as useful as one that passes.

## What changed (what you are testing)

Compared with v7-beta7, the firmware now:

1. sends polls with the correct checksum (v7-beta7's were all wrong)
2. no longer clears every module's ID at boot, and has a command to assign IDs: `PHEVASSIGN=1`
3. reads the module status / error frames from the right CAN IDs
4. shows all 16 cells per module (v7-beta7 showed cells 7–16 as 0.000 V)
5. stops polling at full rate when nothing answers (avoids CAN bus-off)

## What you need

- M5Dial with this test firmware, CAN transceiver on Grove B (see Operating Manual 2.4)
- BMW PHEV modules with their CSC boards and harness, and the low-voltage supply for them
- A laptop with a USB cable and a serial terminal (115200 baud); Arduino Serial Monitor,
  PuTTY or `pio device monitor` all work
- A multimeter that can measure cell voltages at the module's cell taps
- Optional but very helpful: [SavvyCAN](https://www.savvycan.com/) to capture CAN traffic over WiFi

## Safety

- Work on the low-voltage side only. Do not open the module stack or touch HV connections.
- **Check the supply voltage your CSCs need before powering them.** The Operating Manual
  (section 2.5) lists 5 V for SP06/SP41 from the openinverter wiki. Some packs may differ.
  Applying the wrong voltage can destroy a CSC. If you are not sure, ask before powering.
- Unplug the car's own BMS master from the module CAN bus. Two masters on the bus will conflict.
- The firmware does not balance BMW PHEV modules, so it will not draw current from cells.

## 1. Flash and set up

1. Flash the test firmware. Easiest, in Chrome or Edge:
   **https://robertwa1974.github.io/M5Dial-BMS/test/fix-phev-cells-temps/** → *Click to Install TEST build*.
   The page shows the commit it was built from; include it in your report. It always has
   the latest build of this branch. Alternatively download `factory.bin` from that page and:
   - in Chrome/Edge: https://espressif.github.io/esptool-js/ → Connect → flash address `0x0` → Program, or
   - `esptool.py --chip esp32s3 write_flash 0x0 factory.bin`
2. Open the serial terminal at 115200 baud. Press Enter, then type `h` for the menu.
3. Send these commands, one per line:
   ```
   LOGLEVEL=1
   WIFI=1
   CMUTYPE=4
   ```
   `CMUTYPE=4` reboots the unit, which also applies `WIFI=1`. (Holding the encoder
   button for 1 s also toggles WiFi.) `LOGLEVEL=1` keeps the log readable. Use
   `LOGLEVEL=0` only if asked: it prints every CAN frame.
4. Note the firmware version banner at the top of the log (`M5DialBMS v7 booting...`).

## 2. Test A — empty bus (no modules)

With the transceiver connected but **no modules** (CAN bus terminated, or the module
connector unplugged), reboot and watch the log for 1 minute.

- **Pass:** no repeating `CAN: bus-off detected, recovering...` messages.
- Record: pass or fail, and paste any bus-off lines.

## 3. Test B — modules as removed from the car

Connect the modules, power their supply, then reboot the M5Dial. Wait 30 seconds.

- **Pass:** the pack page shows `Mods: N` (6 for SP06/SP41, 12 for SP44), and each module page shows 16 cell voltages and T1/T2.
- Measure **every cell of at least one module** with the multimeter and compare with the display. Expect agreement within about ±10 mV.
- Check the temperatures are plausible (about room temperature).
- Watch for log lines like `PHEV module 3: error word 0x00000000  balstat 0x0000`. Copy them all; non-zero values are important.
- If no modules appear after 30 s, go to Test C.

## 4. Test C — assign module IDs

Do this if Test B showed no modules, or if v7-beta7 or earlier was ever run with these
modules connected (that version erased their IDs).

1. With all modules powered and connected, send:
   ```
   PHEVASSIGN=1
   ```
2. **Expected log:**
   ```
   PHEV: resetting all CSC IDs and assigning new ones...
   PHEV: assigning ID 0 to CSC 1122334455667788
   PHEV: assigning ID 1 to CSC ...
   ...
   PHEV: ID assignment finished - 6 CSC(s) assigned
   ```
3. Within a few seconds the modules should appear (`Mods: N`). Repeat the checks from Test B.
4. IDs go out in the order modules answer, which may not match their physical order.
   Note which display module number matches which physical module, for example by
   comparing a module's total voltage with the meter.
5. **Persistence check:** power off the modules and the M5Dial, power them back on, and
   reboot **without** running `PHEVASSIGN=1` again. Do the modules come back? Record yes or no.

## 5. Test D — 15-minute run

Leave everything running for 15 minutes with the pack page or a module page shown.

- Does `Mods: N` stay constant? Record any module that drops out or reappears.
- Do any cells read 0.000 V or jump around?
- For 8-cell modules (SP44), cells 9–16 read 0 V by design: set the per-module cell
  override to **8** in the web Settings tab (`http://192.168.4.1`) and confirm they disappear.

## 6. Test E — CAN capture (optional, very useful)

1. Join the WiFi `M5DialBMS` (password `M5DialBMS`).
2. In SavvyCAN, add a network connection: host `192.168.4.1`, port `23`, device type `GVRET (ESP32)` (Operating Manual 6.4).
3. Capture 60 seconds starting from an M5Dial reboot, and a second capture while running `PHEVASSIGN=1` (if you did Test C).
4. Save each as CSV.

## What to send back

Please attach to the GitHub pull request or issue, or send to the maintainer:

| Item | Detail |
|---|---|
| Pack | Vehicle and model year, pack part number if visible, number of modules, cells per module |
| Supply | CSC supply voltage used, and how it is wired |
| Firmware | The commit shown on the test installer page (or the `.bin` file name) |
| Serial log | Everything from boot, for each test |
| Multimeter readings | Cell-by-cell table for at least one module, next to the displayed values |
| Photos | Pack page and one module page |
| Results | The table below |
| Captures | SavvyCAN CSV files, if made |

| Test | Pass / fail | Notes |
|---|---|---|
| A — empty bus, no bus-off | | |
| B — modules appear without PHEVASSIGN | | modules seen: |
| B — cells match meter (±10 mV) | | |
| B — temperatures plausible | | |
| C — PHEVASSIGN assigns all modules | | modules assigned: |
| C — modules return after power cycle without PHEVASSIGN | | |
| D — stable for 15 min | | |
| Error word / balstat values logged | | values: |

## Known limitations of this test build

- The 0x373 CAN frame sends the average temperature, not per-module min/max ([issue #4](https://github.com/robertwa1974/M5Dial-BMS/issues/4)).
- On the round display, the corner cells of the 16-cell module page (C1, C4, C13, C16) may be partly cut off. The web page shows all 16 cells.
- A module with a non-zero error word shows `FAULT:0x01`. We do not yet know what the PHEV error bits mean, which is why we ask for the logged values.
- BMW PHEV balancing is not implemented.
