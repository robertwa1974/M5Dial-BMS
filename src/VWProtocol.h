#pragma once
// =============================================================================
// VWProtocol.h - VW CMU (MEB / MQB) CAN frame decoding and building
//
// Pure functions: no Arduino, FreeRTOS or TWAI calls, so the decoders can be
// unit-tested on the host (test/vw_decode_test.cpp).
//
// Ported from Tom de Bree's VWBMSV2 (credit: Tom de Bree / Tom-evnut):
//   https://github.com/Tom-evnut/VW-bms  commit 4a6a5b9 (2024-03-29)
// Citations are "file:line" inside VWBMSV2/ at that commit. Protocol overview
// and the CAN ID constants are in bms_config.h.
//
// Deliberate deviations from the reference (each noted at its function):
//   - Module numbers are bounds-checked against our arrays. The reference's
//     decodetemp() has "if (CMU > 0 && CMU < 15);" - the stray semicolon
//     makes that check a no-op.
//   - Cell frames are length-checked (FlexCAN buffers are always 8 bytes;
//     TWAI reports the real DLC).
//   - Absent temperature sensors are VW_TEMP_INVALID instead of 0 C, so they
//     are filtered by IgnoreTempThresh instead of reading as a real 0 C.
// =============================================================================
#include <stdint.h>
#include <string.h>
#include "bms_config.h"

// ---------------------------------------------------------------------------
// Poll frames (VWBMSV2.ino:3104-3129 sendcommand()) - sent 1 ms apart on
// VW_POLL_ID (standard ID).
// ---------------------------------------------------------------------------
static const uint8_t VW_POLL_FRAME_1[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t VW_POLL_FRAME_2[8] = { 0x45, 0x01, 0x28, 0x00, 0x00, 0x00, 0x00, 0x30 };

// ---------------------------------------------------------------------------
// vwCellIdToModule - map a STANDARD cell-frame ID to module and sub-frame
//
// The reference's 57-entry case table (BMSModuleManager.cpp:430-718) is
// exactly:
//   modules 1-9  : 0x1B0 + 4*(m-1)  + sub, sub 0..3
//   modules 10-16: 0x1D4 + 4*(m-10) + sub, sub 0..2
// i.e. off = id - 0x1B0, module = off/4 + 1, sub = off%4, with sub 3 absent
// on modules 10-16 (0x1D7, 0x1DB, ... 0x1EB are not in the table).
// The test checks this formula against the table extracted from the source.
// Returns false if the ID is not a VW cell frame.
// ---------------------------------------------------------------------------
inline bool vwCellIdToModule(uint32_t id, int &module, int &sub)
{
    if (id < VW_CELL_ID_MIN || id > VW_CELL_ID_MAX) return false;
    uint32_t off = id - VW_CELL_ID_MIN;
    module = (int)(off / 4) + 1;
    sub    = (int)(off % 4);
    if (module > VW_STD_MODS && sub > 2) return false;   // extender: no sub-frame 3
    return true;
}

// ---------------------------------------------------------------------------
// vwCellFrameValid - init/spoof guards (BMSModuleManager.cpp:687-702)
//   sub 0-1: drop if buf[2], buf[5] or buf[7] == 0xFF
//   sub 2-3: drop if buf[2] == 0xFF
// Plus a DLC check (deviation): sub 0-2 need all 8 bytes, sub 3 needs 3.
// ---------------------------------------------------------------------------
inline bool vwCellFrameValid(int sub, const uint8_t *buf, uint8_t dlc)
{
    if (dlc < ((sub == 3) ? 3 : 8)) return false;
    if (sub < 2) return buf[2] != 0xFF && buf[5] != 0xFF && buf[7] != 0xFF;
    return buf[2] != 0xFF;
}

// ---------------------------------------------------------------------------
// vwDecodeCells - unpack cell voltages in mV (BMSModule.cpp:99-240)
//
// 12-bit nibble-packed, mV = raw + 1000. For sub-frame n, cells 4n..4n+3:
//   cell[4n+0] = (buf[1] >> 4) | (buf[2] << 4)
//   cell[4n+1] =  buf[3]       | ((buf[4] & 0x0F) << 8)
//   cell[4n+2] = (buf[4] >> 4) | (buf[5] << 4)
//   cell[4n+3] =  buf[6]       | ((buf[7] & 0x0F) << 8)
// Sub-frame 3 carries only cell 13 (index 12), packed like cell[4n+0]
// (BMSModule.cpp:231-234).
// Writes mv[0..3] and returns the number of cells decoded (4, or 1 for sub 3).
// ---------------------------------------------------------------------------
inline int vwDecodeCells(int sub, const uint8_t *buf, uint16_t *mv)
{
    mv[0] = (uint16_t)((((uint16_t)buf[1] >> 4) | ((uint16_t)buf[2] << 4)) + 1000);
    if (sub == 3) return 1;
    mv[1] = (uint16_t)(((uint16_t)buf[3] | ((uint16_t)(buf[4] & 0x0F) << 8)) + 1000);
    mv[2] = (uint16_t)((((uint16_t)buf[4] >> 4) | ((uint16_t)buf[5] << 4)) + 1000);
    mv[3] = (uint16_t)(((uint16_t)buf[6] | ((uint16_t)(buf[7] & 0x0F) << 8)) + 1000);
    return 4;
}

// ---------------------------------------------------------------------------
// Presence and cell count from sub-frames (deviation - the reference writes
// cells as they arrive and never drops a module).
//
// A module counts as present once sub-frames 0 and 1 (cells 1-8) have both
// arrived since the last presence update, so an 8s MEB module that never
// sends a valid sub-frame 2 still appears. Cells 9-13 are filled in as their
// sub-frames arrive.
// The cell count follows the highest sub-frame ever received:
//   sub-frame 3 seen -> 13 cells, sub-frame 2 seen -> 12, otherwise 8.
// (A per-module cell override in settings takes precedence.)
// ---------------------------------------------------------------------------
#define VW_PRESENCE_SUBFRAMES   0x03    // sub-frames 0 and 1

inline bool vwPresenceComplete(uint8_t framesRx)
{
    return (framesRx & VW_PRESENCE_SUBFRAMES) == VW_PRESENCE_SUBFRAMES;
}

inline int vwCellsFromSubFrames(uint8_t subSeen)
{
    if (subSeen & 0x08) return 13;
    if (subSeen & 0x04) return 12;
    return 8;
}

// ---------------------------------------------------------------------------
// Temperature results
// The reference only overwrites the sensors a frame actually carries; the
// others keep their previous value. VW_T_UNCHANGED models that.
// ---------------------------------------------------------------------------
enum VwTempState : uint8_t {
    VW_T_UNCHANGED = 0,   // frame does not carry this sensor - keep old value
    VW_T_VALID     = 1,   // t[] holds a new reading
    VW_T_ABSENT    = 2,   // sensor reported absent (reference sets 0 C)
};

struct VwTempResult {
    float    t[VW_TEMPS_PER_MOD];
    uint8_t  state[VW_TEMPS_PER_MOD];
    bool     hasBalstat;     // type 1 frames can also carry balance status
    uint32_t balstat;
};

// ---------------------------------------------------------------------------
// vwTemp1Module - module number for a type 1 temperature frame
// (BMSModuleManager.cpp:391-395):
//   CMU = id & 0xFF; if 10 < CMU < 60: CMU = (CMU & 0x0F) * 0.5 + 1 (truncated)
//   otherwise CMU = low byte unchanged.
// Returns 0 if the result is outside 1..VW_MAX_MODS (the reference's own
// bounds check is a no-op - see header).
// ---------------------------------------------------------------------------
inline int vwTemp1Module(uint32_t id)
{
    int cmu = (int)(id & 0xFF);
    if (cmu > 10 && cmu < 60) {
        cmu = (cmu & 0x0F) / 2 + 1;   // == (int)((cmu & 0x0F) * 0.5 + 1) for cmu >= 0
    }
    return (cmu >= 1 && cmu <= VW_MAX_MODS) ? cmu : 0;
}

// ---------------------------------------------------------------------------
// vwDecodeTemp1 - type 1 temperature frame (BMSModule.cpp:53-91)
//   buf[7] == 0xFD:  T0 = buf[2]*0.5 - 40, unless buf[2] == 0xFD
//   otherwise:
//     buf[0] < 0xDF: T0 = buf[0]*0.5 - 43, balstat = buf[2] | buf[3] << 8
//     else:          T0 = buf[3]*0.5 - 43
//     T1 = buf[4]*0.5 - 43 if buf[4] < 0xF0, else absent
//     T2 = buf[5]*0.5 - 43 if buf[5] < 0xF0, else absent
// Returns false if the frame is too short to decode.
// ---------------------------------------------------------------------------
inline bool vwDecodeTemp1(const uint8_t *buf, uint8_t dlc, VwTempResult &r)
{
    memset(&r, 0, sizeof(r));
    if (dlc < 8) return false;
    if (buf[7] == 0xFD) {
        if (buf[2] != 0xFD) {
            r.t[0] = buf[2] * 0.5f - 40.0f;
            r.state[0] = VW_T_VALID;
        }
        return true;
    }
    if (buf[0] < 0xDF) {
        r.t[0] = buf[0] * 0.5f - 43.0f;
        r.hasBalstat = true;
        r.balstat = (uint32_t)buf[2] + ((uint32_t)buf[3] << 8);
    } else {
        r.t[0] = buf[3] * 0.5f - 43.0f;
    }
    r.state[0] = VW_T_VALID;
    if (buf[4] < 0xF0) { r.t[1] = buf[4] * 0.5f - 43.0f; r.state[1] = VW_T_VALID; }
    else               { r.t[1] = VW_TEMP_INVALID;       r.state[1] = VW_T_ABSENT; }
    if (buf[5] < 0xF0) { r.t[2] = buf[5] * 0.5f - 43.0f; r.state[2] = VW_T_VALID; }
    else               { r.t[2] = VW_TEMP_INVALID;       r.state[2] = VW_T_ABSENT; }
    return true;
}

// ---------------------------------------------------------------------------
// vwTemp2Module - module number for a type 2 (MEB) temperature frame
// (BMSModuleManager.cpp:410-414): (id & 0x0F) + 1.
// Returns 0 if outside 1..VW_MAX_MODS.
// ---------------------------------------------------------------------------
inline int vwTemp2Module(uint32_t id)
{
    int cmu = (int)(id & 0x0F) + 1;
    return (cmu >= 1 && cmu <= VW_MAX_MODS) ? cmu : 0;
}

// ---------------------------------------------------------------------------
// vwDecodeTemp2 - type 2 (MEB) temperature frame
//   skip if buf[5] == 0xDF (module initialising / "spoof module",
//     BMSModuleManager.cpp:415)
//   T0 = (((buf[5] & 0x0F) << 4) | (buf[4] >> 4)) * 0.5 - 40
//     (BMSModule.cpp:95, "MEB Bits 36-44")
// Returns false if the frame is skipped.
// ---------------------------------------------------------------------------
inline bool vwDecodeTemp2(const uint8_t *buf, uint8_t dlc, VwTempResult &r)
{
    memset(&r, 0, sizeof(r));
    if (dlc < 6) return false;
    if (buf[5] == 0xDF) return false;
    uint16_t raw = (uint16_t)(((buf[5] & 0x0F) << 4) | ((buf[4] & 0xF0) >> 4));
    r.t[0] = raw * 0.5f - 40.0f;
    r.state[0] = VW_T_VALID;
    return true;
}

// ---------------------------------------------------------------------------
// vwBalstatModule - module number for an MEB balance-status frame
// (BMSModuleManager.cpp:353-357): id & 0x0F, accepted when 1..14.
// Note: unlike type 2 temperatures there is no +1 here. Returns 0 if rejected.
// ---------------------------------------------------------------------------
inline int vwBalstatModule(uint32_t id)
{
    int cmu = (int)(id & 0x0F);
    return (cmu > 0 && cmu < 15 && cmu <= VW_MAX_MODS) ? cmu : 0;
}

// ---------------------------------------------------------------------------
// vwDecodeBalstat - MEB balance status (BMSModule.cpp:46-49)
//   balstat = (buf[1] >> 4) | (buf[2] << 4) | (buf[3] << 12)
// (reference balstat is uint32_t, so all of buf[3] is kept)
// ---------------------------------------------------------------------------
inline bool vwDecodeBalstat(const uint8_t *buf, uint8_t dlc, uint32_t &balstat)
{
    if (dlc < 4) return false;
    balstat = ((uint32_t)buf[1] >> 4) | ((uint32_t)buf[2] << 4) | ((uint32_t)buf[3] << 12);
    return true;
}

// ---------------------------------------------------------------------------
// Balance command IDs (BMSModuleManager.cpp:110-210), EXTENDED, modules 1-12:
//   modules 1-9  : A = 0x1A55540A + 2*(m-1), B = A + 1
//   modules 10-12: A = 0x1A5554AB / AD / AF,  B = A + 1
// Returns 0 for modules outside 1..12 (deviation: the reference's switch has
// "default: break;" and then still transmits, re-using the previous module's
// ID for modules > 12).
// ---------------------------------------------------------------------------
inline uint32_t vwBalCmdIdA(int m)
{
    if (m >= 1 && m <= 9)               return 0x1A55540AUL + 2UL * (uint32_t)(m - 1);
    if (m >= 10 && m <= VW_MAX_BAL_MODS) return 0x1A5554ABUL + 2UL * (uint32_t)(m - 10);
    return 0;
}

inline uint32_t vwBalCmdIdB(int m)
{
    uint32_t a = vwBalCmdIdA(m);
    return a ? a + 1 : 0;
}

// ---------------------------------------------------------------------------
// vwBuildBalFrames - balance command payloads (BMSModuleManager.cpp:96-165)
//   frame A: buf[i]   = 0x08 if cell i (0-7) bleeds, else 0x00
//   frame B: buf[i-8] = 0x08 if cell i (8-12) bleeds, else 0x00; buf[5..7] = 0xFE
// ---------------------------------------------------------------------------
inline void vwBuildBalFrames(uint16_t mask, uint8_t *a, uint8_t *b)
{
    for (int i = 0; i < 8; i++)  a[i]     = (mask & (1u << i)) ? 0x08 : 0x00;
    for (int i = 8; i < 13; i++) b[i - 8] = (mask & (1u << i)) ? 0x08 : 0x00;
    b[5] = 0xFE;
    b[6] = 0xFE;
    b[7] = 0xFE;
}

// ---------------------------------------------------------------------------
// vwBuildBalStopFrames - "stop balancing" payloads sent to all 12 module ID
// pairs at balcnt == 1 (BMSModuleManager.cpp:223-343)
//   frame A: 00 00 00 00 00 00 00 00
//   frame B: 00 00 00 00 FE FE FE FE
// ---------------------------------------------------------------------------
inline void vwBuildBalStopFrames(uint8_t *a, uint8_t *b)
{
    memset(a, 0x00, 8);
    memset(b, 0x00, 4);
    memset(b + 4, 0xFE, 4);
}

// ---------------------------------------------------------------------------
// vwBalanceGate - start condition (VWBMSV2.ino:492):
//   high cell > balanceVoltage  AND  high cell > low cell + hysteresis
// ---------------------------------------------------------------------------
inline bool vwBalanceGate(float highCell, float lowCell, float balanceV, float hyst)
{
    return highCell > balanceV && highCell > lowCell + hyst;
}

// ---------------------------------------------------------------------------
// vwBalanceMask - cells to bleed (BMSModuleManager.cpp:72-78):
//   bit i set when cell i > lowest pack cell + hysteresis (up to 13 cells)
// ---------------------------------------------------------------------------
inline uint16_t vwBalanceMask(const float *cellV, int numCells, float lowCell, float hyst)
{
    uint16_t mask = 0;
    for (int i = 0; i < numCells && i < 13; i++)
        if ((lowCell + hyst) < cellV[i]) mask |= (uint16_t)(1u << i);
    return mask;
}

// ---------------------------------------------------------------------------
// vwBalanceAction - balance duty-cycle state machine (BMSModuleManager.cpp:62-68,
// 223, 345). Called once per VW_BAL_TICK_MS with the running counter:
//   counter > 60       -> wrap to 0
//   counter == 1       -> STOP: send stop frames, resume cell measurement
//   counter 2..10      -> IDLE: measuring (cell decode active)
//   counter 11, 15, 20 ... 55 -> SEND: recompute and send balance masks
//   anything else      -> IDLE: modules keep bleeding
// The caller then: on SEND with no cell needing balance, sets counter = 0
// (BMSModuleManager.cpp:218-220); always increments counter afterwards.
// ---------------------------------------------------------------------------
enum VwBalAction : uint8_t { VW_BAL_IDLE = 0, VW_BAL_STOP = 1, VW_BAL_SEND = 2 };

inline VwBalAction vwBalanceAction(uint8_t &counter)
{
    if (counter > 60) counter = 0;
    if (counter > 10) {
        if (counter == 11 || (counter >= 15 && counter <= 55 && counter % 5 == 0))
            return VW_BAL_SEND;
        return VW_BAL_IDLE;
    }
    return (counter == 1) ? VW_BAL_STOP : VW_BAL_IDLE;
}
