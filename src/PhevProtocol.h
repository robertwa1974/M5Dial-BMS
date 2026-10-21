#pragma once
// =============================================================================
// PhevProtocol.h - BMW PHEV CSC (SP06 / SP41 / SP44) CAN frame building and
// decoding
//
// Pure functions: no Arduino, FreeRTOS or TWAI calls, so they can be
// unit-tested on the host (test/phev_protocol_test.cpp).
//
// Reference: Tom de Bree's BMWPhevBMS (credit: Tom de Bree / Tom-evnut)
//   https://github.com/Tom-evnut/BMWPhevBMS  commit 91a7b12 (2024-04-26)
// Citations are "file:line" at that commit.
// =============================================================================
#include <stdint.h>
#include <string.h>

#define PHEV_POLL_ID_BASE      0x080   // TX: poll, 0x080 | slot (BMWPhevBMS.ino:3352)
#define PHEV_MGMT_ID           0x0A0   // TX: ID management broadcast (BMWPhevBMS.ino:3668)
#define PHEV_UNASSIGNED_ID     0x0F0   // RX: reply from a CSC with no ID (BMWPhevBMS.ino:3031)
#define PHEV_MAX_SLOTS         12      // finalxor table size (BMWPhevBMS.ino:226)
#define PHEV_COUNTER_WRAP      0x0F    // rolling counter runs 0..14 (BMWPhevBMS.ino:3331)

// Final XOR per slot, from BMWPhevBMS.ino:226 (also defined in CANManager.cpp
// as BMW_PHEV_FINAL_XOR for the existing callers)
static const uint8_t PHEV_FINAL_XOR[PHEV_MAX_SLOTS] = {
    0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C
};

// ---------------------------------------------------------------------------
// phevCrc8 - CRC-8, polynomial 0x1D, init 0xFF, XOR with finalXor
// (CRC8.cpp / CRC8.h in the reference: table-driven, same result)
// ---------------------------------------------------------------------------
inline uint8_t phevCrc8(const uint8_t *data, int len, uint8_t finalXor)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
    }
    return crc ^ finalXor;
}

// ---------------------------------------------------------------------------
// phevPollChecksum - getcheck() (BMWPhevBMS.ino:3629-3647)
// CRC over 9 bytes: id high, id low, buf[0..6]. buf[6] holds the rolling
// counter, so it must be covered. (v7-beta7 and earlier covered only
// buf[0..5]: every poll had a wrong CRC.)
// ---------------------------------------------------------------------------
inline uint8_t phevPollChecksum(uint32_t id, const uint8_t *buf, int slot)
{
    uint8_t m[9];
    m[0] = (uint8_t)(id >> 8);
    m[1] = (uint8_t)(id & 0xFF);
    for (int i = 0; i < 7; i++) m[i + 2] = buf[i];
    return phevCrc8(m, 9, PHEV_FINAL_XOR[slot]);
}

// ---------------------------------------------------------------------------
// phevBuildPoll - one poll frame (BMWPhevBMS.ino:3352-3382)
//   buf[0-1]: balance target mV + 5 (LE) when balancing, else C7 10
//   buf[2-3]: 00 00
//   buf[4-5]: 20 00 while testCycle < 3, else 40 01 (48 01 when balancing)
//   buf[6]:   counter << 4, plus 0x04 when testCycle == 2
//   buf[7]:   phevPollChecksum()
// Returns the CAN ID (0x080 | slot).
// ---------------------------------------------------------------------------
inline uint32_t phevBuildPoll(int slot, uint8_t testCycle, uint8_t counter,
                              bool balance, uint16_t balTargetMv, uint8_t *buf)
{
    uint32_t id = (uint32_t)PHEV_POLL_ID_BASE | (uint32_t)slot;
    if (balance) {
        buf[0] = (uint8_t)(balTargetMv & 0xFF);
        buf[1] = (uint8_t)(balTargetMv >> 8);
    } else {
        buf[0] = 0xC7;
        buf[1] = 0x10;
    }
    buf[2] = 0x00;
    buf[3] = 0x00;
    if (testCycle < 3) {
        buf[4] = 0x20;
        buf[5] = 0x00;
    } else {
        buf[4] = balance ? 0x48 : 0x40;
        buf[5] = 0x01;
    }
    buf[6] = (uint8_t)(counter << 4);
    if (testCycle == 2) buf[6] = (uint8_t)(buf[6] + 0x04);
    buf[7] = phevPollChecksum(id, buf, slot);
    return id;
}

// ---------------------------------------------------------------------------
// phevAdvance - poll sequencer after one frame (BMWPhevBMS.ino:3324-3335)
// After the last slot: counter++ (wrapping 0x0F -> 0, i.e. 0..14 - the
// reference never sends 0xF), testCycle++ up to 4.
// The reference cycles 6 slots; this firmware cycles numSlots (12).
// ---------------------------------------------------------------------------
inline void phevAdvance(uint8_t &slot, uint8_t &counter, uint8_t &testCycle, int numSlots)
{
    slot++;
    if (slot >= numSlots) {
        slot = 0;
        counter++;
        if (counter >= PHEV_COUNTER_WRAP) counter = 0;
        if (testCycle < 4) testCycle++;
    }
}

// ---------------------------------------------------------------------------
// ID management frames, all on PHEV_MGMT_ID (0x0A0), standard ID
// ---------------------------------------------------------------------------
// Reset one ID: A1 <id> FF FF FF FF FF FF (resetIDdebug, BMWPhevBMS.ino:3665-3683)
inline void phevBuildResetId(uint8_t id, uint8_t *buf)
{
    buf[0] = 0xA1;
    buf[1] = id;
    memset(buf + 2, 0xFF, 6);
}

// Find unassigned CSCs: 37 FF FF FF FF FF FF FF (findUnassigned, :3704-3720)
// Each CSC without an ID answers on PHEV_UNASSIGNED_ID with its 8-byte DMC.
inline void phevBuildFindUnassigned(uint8_t *buf)
{
    buf[0] = 0x37;
    memset(buf + 1, 0xFF, 7);
}

// Assign nextId to the CSC whose DMC was received (assignID, :3722-3760).
// Four frames, sent in order with 30 / 10 / 10 ms gaps:
//   f[0] = 12 AB DMC0 DMC1 DMC2 DMC3 FF FF
//   f[1] = 12 BA DMC4 DMC5 DMC6 DMC7 FF FF
//   f[2] = 5B id DMC4 DMC5 DMC6 DMC7 FF FF   (bytes 2-7 carried over from f[1])
//   f[3] = 37 id DMC4 DMC5 DMC6 DMC7 FF FF
// followed by a find-unassigned broadcast for the next CSC.
inline void phevBuildAssign(const uint8_t *dmc, uint8_t nextId, uint8_t f[4][8])
{
    f[0][0] = 0x12; f[0][1] = 0xAB;
    f[0][2] = dmc[0]; f[0][3] = dmc[1]; f[0][4] = dmc[2]; f[0][5] = dmc[3];
    f[0][6] = 0xFF; f[0][7] = 0xFF;
    memcpy(f[1], f[0], 8);
    f[1][1] = 0xBA;
    f[1][2] = dmc[4]; f[1][3] = dmc[5]; f[1][4] = dmc[6]; f[1][5] = dmc[7];
    memcpy(f[2], f[1], 8);
    f[2][0] = 0x5B; f[2][1] = nextId;
    memcpy(f[3], f[2], 8);
    f[3][0] = 0x37; f[3][1] = nextId;
}

// ---------------------------------------------------------------------------
// RX classification (BMWPhevBMS.ino:3061-3076, BMSModuleManager.cpp:94-150)
//   0x100-0x10F  status: bytes 0-3 error word LE, bytes 4-5 balstat LE
//                (id & 0x0F0 == 0x000; v7-beta7 looked at 0x0A0 instead)
//   0x120-0x17F  cells: id & 0x0F0 = 0x020..0x070 -> sub-frame 0..5
//   0x180-0x18F  temperatures: 4 x uint8, value - 40 = deg C
//   module number = (id & 0x00F) + 1
// ---------------------------------------------------------------------------
enum PhevFrameKind : uint8_t {
    PHEV_RX_NONE = 0, PHEV_RX_STATUS, PHEV_RX_CELLS, PHEV_RX_TEMPS, PHEV_RX_UNASSIGNED
};

inline PhevFrameKind phevClassify(uint32_t id, bool extd, int &module, int &sub)
{
    module = 0;
    sub    = 0;
    if (extd) return PHEV_RX_NONE;
    if (id == PHEV_UNASSIGNED_ID) return PHEV_RX_UNASSIGNED;
    if (id < 0x100 || id > 0x18F) return PHEV_RX_NONE;
    int m = (int)(id & 0x00F) + 1;
    if (m > PHEV_MAX_SLOTS) return PHEV_RX_NONE;   // no poll slot / finalxor beyond 12
    module = m;
    uint32_t type = id & 0xFF0;
    if (type == 0x100) return PHEV_RX_STATUS;
    if (type == 0x180) return PHEV_RX_TEMPS;
    if (type >= 0x120 && type <= 0x170) {
        sub = (int)((type - 0x120) >> 4);   // 0..5
        return PHEV_RX_CELLS;
    }
    return PHEV_RX_NONE;                    // 0x110-0x11F: not used by the reference
}

// Cells in a sub-frame (BMSModule.cpp:59-130): 3 cells for sub 0-4, 1 for
// sub 5 (cell 16). LE 14-bit: mV = buf[2n] + (buf[2n+1] & 0x3F) * 256.
// Writes mv[] and returns the number of cells; first cell index = sub * 3.
inline int phevDecodeCells(int sub, const uint8_t *buf, uint16_t *mv)
{
    int n = (sub == 5) ? 1 : 3;
    for (int c = 0; c < n; c++)
        mv[c] = (uint16_t)(buf[c * 2] + (uint16_t)(buf[c * 2 + 1] & 0x3F) * 256);
    return n;
}

// Presence: once sub-frames 0-2 (cells 1-9) have arrived since the last
// update, so a module with 8 cells appears even if it never sends the upper
// sub-frames. (v7-beta7 waited for all six.) Deviation: the reference has no
// presence timeout - it writes cells as they arrive.
#define PHEV_PRESENCE_SUBFRAMES 0x07
inline bool phevPresenceComplete(uint8_t framesRx)
{
    return (framesRx & PHEV_PRESENCE_SUBFRAMES) == PHEV_PRESENCE_SUBFRAMES;
}
