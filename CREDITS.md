# Credits

## Original Firmware

Original TeslaBMS firmware by **Collin Kidder**:
- https://github.com/collin80/TeslaBMS
- Licence: MIT

SimpBMS firmware by **Tom-evnut**
-https://github.com/Tom-evnut/TeslaBMSV2

The BQ76PL536A register map, CRC algorithm, Steinhart-Hart temperature
conversion, and UART daisy-chain protocol are derived directly from
Collin Kidder's original work and are used in accordance with his MIT licence.

## ESP32-S3 Port

ESP32-S3 port by **ronaegis**, used as the direct starting point for this project:
- https://github.com/ronaegis/tesla-bms-esp32s3

## BMW i3 CAN Protocol Reference

BMW i3 CSC and BMW PHEV CAN protocol research by **Tom-evnut**:
- https://github.com/Tom-evnut/TeslaBMSV2

## VW CMU (MEB / MQB) CAN Protocol Reference

VW battery module CMU protocol (cell voltages, temperatures, balance status,
balance commands and the 0x0BA poll) ported from **Tom de Bree (Tom-evnut)**'s
VWBMSV2, with his permission:
- https://github.com/Tom-evnut/VW-bms (commit 4a6a5b9)

The VW decoders in `src/VWProtocol.h` cite the reference file and line for
every ported rule.

## BMW i3 BUS_variant CAN Protocol Reference

BMW i3 CSC CAN protocol research by **hpeyerl**:
- https://github.com/hpeyerl/bmw-i3-bms-tcan485

## Licence

This project is released under the MIT Licence in keeping with the original.
The original copyright notice and licence text from Collin Kidder's TeslaBMS
must be preserved in any distribution or derivative work.
