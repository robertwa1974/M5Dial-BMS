#pragma once
// =============================================================================
// PacificaProtocol.h - Chrysler Pacifica PHEV BPCM CAN frame decoding
//
// Pure functions: no Arduino, FreeRTOS or TWAI calls, so the decoders can be
// unit-tested on the host (test/pacifica_decode_test.cpp).
//
// Source: the SavvyCAN DBC posted by Slyd for a 2023 Pacifica pack powered on
// the bench (diyelectriccar.com, "Tesla Motor + Chrysler Pacifica BPCM in
// Smart Fortwo ED", post #16, 2025-09-10). Not yet confirmed on hardware here,
// and earlier pack years (2017-2018, part families 05193143 / 68394027) may
// use a different map. CAN ID constants are in bms_config.h.
//
//   cells   0x487-0x49E  module m (1..6) sub s (0..3): id = 0x487 + 4*(m-1) + s
//                        4 cells per frame, 16-bit big-endian, 1 mV/bit
//                        (DBC: 7|16@0+, 23|16@0+, 39|16@0+, 55|16@0+, x0.001 V)
//   temps   0x4A0        M1T1 M1T2 M2T1 M2T2 M3T1 M3T2, one byte each, -40 C
//           0x4A1        same for modules 4-6 (DLC 6)
//   coolant 0x307        byte 0 outlet, byte 1 inlet, -40 C
//   status  0x150        HVIL_Stat bit 43, HVIL_External_Loop bit 30,
//                        11-bit field 26|11 x0.192 that Slyd labels SOC
//                        (0-393 range: looks more like pack voltage)
//
// No balancing or keep-alive commands are known: the BPCM is described as
// balancing on its own. Nothing here transmits.
// =============================================================================
#include <stdint.h>
#include "bms_config.h"

// ---------------------------------------------------------------------------
// pacCellIdToModule - map a cell-frame ID to module (1..6) and sub-frame (0..3)
// Returns false if the ID is not a Pacifica cell frame.
// ---------------------------------------------------------------------------
inline bool pacCellIdToModule(uint32_t id, int &module, int &sub)
{
    if (id < PAC_CELL_ID_MIN || id > PAC_CELL_ID_MAX) return false;
    uint32_t off = id - PAC_CELL_ID_MIN;
    module = (int)(off / 4) + 1;
    sub    = (int)(off % 4);
    return true;
}

// ---------------------------------------------------------------------------
// pacDecodeCells - 4 big-endian mV values from a cell frame.
// Needs a full 8-byte frame. Returns the number of cells decoded (0 or 4).
// ---------------------------------------------------------------------------
inline int pacDecodeCells(const uint8_t *buf, uint8_t dlc, uint16_t mv[4])
{
    if (dlc < 8) return 0;
    for (int k = 0; k < 4; k++)
        mv[k] = (uint16_t)((buf[2 * k] << 8) | buf[2 * k + 1]);
    return 4;
}

// ---------------------------------------------------------------------------
// pacTempIdFirstModule - first module carried by a temperature frame
// (1 for 0x4A0, 4 for 0x4A1), 0 if not a temperature frame.
// ---------------------------------------------------------------------------
inline int pacTempIdFirstModule(uint32_t id)
{
    if (id == PAC_TEMP_ID_A) return 1;
    if (id == PAC_TEMP_ID_B) return 4;
    return 0;
}

// ---------------------------------------------------------------------------
// pacDecodeTemps - 6 temperatures (3 modules x 2 sensors) in deg C.
// t[2*i + j] = module (first + i), sensor j. Needs DLC >= 6.
// ---------------------------------------------------------------------------
inline bool pacDecodeTemps(const uint8_t *buf, uint8_t dlc, float t[6])
{
    if (dlc < 6) return false;
    for (int i = 0; i < 6; i++) t[i] = (float)buf[i] - 40.0f;
    return true;
}

// ---------------------------------------------------------------------------
// pacDecodeCoolant - coolant outlet / inlet temperatures in deg C.
// ---------------------------------------------------------------------------
inline bool pacDecodeCoolant(const uint8_t *buf, uint8_t dlc, float &outlet, float &inlet)
{
    if (dlc < 2) return false;
    outlet = (float)buf[0] - 40.0f;
    inlet  = (float)buf[1] - 40.0f;
    return true;
}

// ---------------------------------------------------------------------------
// pacMotorolaBits - DBC Motorola (big-endian, @0) field extraction.
// start = DBC start bit (MSB of the field), length in bits, from an 8-byte frame.
// ---------------------------------------------------------------------------
inline uint32_t pacMotorolaBits(const uint8_t *buf, int start, int length)
{
    uint64_t u = 0;
    for (int i = 0; i < 8; i++) u = (u << 8) | buf[i];
    int msb   = (start / 8) * 8 + (7 - start % 8);   // MSB position from the left
    int shift = 64 - msb - length;
    if (shift < 0) return 0;
    return (uint32_t)((u >> shift) & ((1ULL << length) - 1));
}

struct PacStatus {
    bool     hvilClosed;       // HVIL_Stat, bit 43 (1 = closed)
    bool     hvilExtClosed;    // HVIL_External_Loop, bit 30 (1 = closed)
    uint16_t field26Raw;       // 26|11 raw; x0.192 per the posted DBC (SOC or pack V)
};

// ---------------------------------------------------------------------------
// pacDecodeStatus - 0x150. Needs a full 8-byte frame.
// ---------------------------------------------------------------------------
inline bool pacDecodeStatus(const uint8_t *buf, uint8_t dlc, PacStatus &s)
{
    if (dlc < 8) return false;
    s.hvilClosed    = pacMotorolaBits(buf, 43, 1) != 0;
    s.hvilExtClosed = pacMotorolaBits(buf, 30, 1) != 0;
    s.field26Raw    = (uint16_t)pacMotorolaBits(buf, 26, 11);
    return true;
}
