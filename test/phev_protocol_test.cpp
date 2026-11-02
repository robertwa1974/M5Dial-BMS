// =============================================================================
// phev_protocol_test.cpp - host-side unit test for the BMW PHEV CSC protocol
// (src/PhevProtocol.h)
//
// Build and run from the repo root with any C++11 host compiler:
//   g++ -std=c++11 -Wall -Wextra -Isrc test/phev_protocol_test.cpp -o phev_protocol_test
//   ./phev_protocol_test
//
// Expected poll frames below were computed independently with the reference's
// table-driven CRC8 and getcheck() byte layout (Tom-evnut/BMWPhevBMS @ 91a7b12:
// CRC8.cpp, BMWPhevBMS.ino:3352-3382, 3629-3647), not with PhevProtocol.h.
// =============================================================================
#include <stdio.h>
#include "PhevProtocol.h"

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_EQ(a, b) do { \
    long long _a = (long long)(a), _b = (long long)(b); \
    if (_a == _b) { g_pass++; } \
    else { g_fail++; printf("FAIL %s:%d  %s == %s  (%lld vs %lld)\n", __FILE__, __LINE__, #a, #b, _a, _b); } \
} while (0)

// Reference CRC8 (table-driven, as in the reference CRC8.cpp)
static uint8_t refCrc8(const uint8_t *m, int n, uint8_t fin)
{
    uint8_t tbl[256];
    for (int d = 0; d < 256; d++) {
        uint8_t r = (uint8_t)d;
        for (int b = 0; b < 8; b++) r = (r & 0x80) ? (uint8_t)((r << 1) ^ 0x1D) : (uint8_t)(r << 1);
        tbl[d] = r;
    }
    uint8_t r = 0xFF;
    for (int i = 0; i < n; i++) r = tbl[m[i] ^ r];
    return r ^ fin;
}

// ---------------------------------------------------------------------------
static void testCrc()
{
    // Bitwise implementation == reference table implementation
    uint8_t m[9];
    for (int trial = 0; trial < 200; trial++) {
        for (int i = 0; i < 9; i++) m[i] = (uint8_t)(trial * 37 + i * 101);
        CHECK_EQ(phevCrc8(m, 9, PHEV_FINAL_XOR[trial % 12]), refCrc8(m, 9, PHEV_FINAL_XOR[trial % 12]));
    }
}

// ---------------------------------------------------------------------------
struct PollVec { int slot; int testCycle; int counter; uint8_t frame[8]; };
static const PollVec POLLS[] = {
    { 0, 0,  0, { 0xC7, 0x10, 0x00, 0x00, 0x20, 0x00, 0x00, 0x25 } },
    { 0, 2,  1, { 0xC7, 0x10, 0x00, 0x00, 0x20, 0x00, 0x14, 0x9C } },
    { 0, 4,  7, { 0xC7, 0x10, 0x00, 0x00, 0x40, 0x01, 0x70, 0x60 } },
    { 0, 4, 14, { 0xC7, 0x10, 0x00, 0x00, 0x40, 0x01, 0xE0, 0x8B } },
    { 5, 0,  0, { 0xC7, 0x10, 0x00, 0x00, 0x20, 0x00, 0x00, 0xC9 } },
    { 5, 2,  1, { 0xC7, 0x10, 0x00, 0x00, 0x20, 0x00, 0x14, 0x70 } },
    { 5, 4,  7, { 0xC7, 0x10, 0x00, 0x00, 0x40, 0x01, 0x70, 0x8C } },
    { 5, 4, 14, { 0xC7, 0x10, 0x00, 0x00, 0x40, 0x01, 0xE0, 0x67 } },
    { 11, 0,  0, { 0xC7, 0x10, 0x00, 0x00, 0x20, 0x00, 0x00, 0x85 } },
    { 11, 2,  1, { 0xC7, 0x10, 0x00, 0x00, 0x20, 0x00, 0x14, 0x3C } },
    { 11, 4,  7, { 0xC7, 0x10, 0x00, 0x00, 0x40, 0x01, 0x70, 0xC0 } },
    { 11, 4, 14, { 0xC7, 0x10, 0x00, 0x00, 0x40, 0x01, 0xE0, 0x2B } },
};

static void testPollFrames()
{
    for (size_t i = 0; i < sizeof(POLLS) / sizeof(POLLS[0]); i++) {
        const PollVec &v = POLLS[i];
        uint8_t buf[8];
        uint32_t id = phevBuildPoll(v.slot, (uint8_t)v.testCycle, (uint8_t)v.counter, false, 0, buf);
        CHECK_EQ(id, 0x080 + v.slot);
        for (int b = 0; b < 8; b++) {
            if (buf[b] != v.frame[b])
                printf("  poll slot %d tc %d cnt %d byte %d: 0x%02X vs 0x%02X\n",
                       v.slot, v.testCycle, v.counter, b, buf[b], v.frame[b]);
            CHECK_EQ(buf[b], v.frame[b]);
        }
    }

    // The v7-beta7 bug: CRC over buf[0..5] only. Must differ from the real one.
    uint8_t buf[8];
    phevBuildPoll(0, 4, 7, false, 0, buf);
    uint8_t m8[8] = { 0x00, 0x80, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5] };
    CHECK(refCrc8(m8, 8, PHEV_FINAL_XOR[0]) != buf[7]);

    // Balancing payload: target mV LE, 0x48 measurement type
    phevBuildPoll(3, 4, 2, true, 3905, buf);
    CHECK_EQ(buf[0], 3905 & 0xFF);
    CHECK_EQ(buf[1], 3905 >> 8);
    CHECK_EQ(buf[4], 0x48);
    CHECK_EQ(buf[5], 0x01);
    uint8_t m9[9] = { 0x00, 0x83, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6] };
    CHECK_EQ(buf[7], refCrc8(m9, 9, PHEV_FINAL_XOR[3]));
}

// ---------------------------------------------------------------------------
static void testSequencer()
{
    uint8_t slot = 0, counter = 0, tc = 0;
    bool sawF = false;
    int maxCounter = 0;
    for (int i = 0; i < 12 * 40; i++) {
        if (counter == 0x0F) sawF = true;
        if (counter > maxCounter) maxCounter = counter;
        phevAdvance(slot, counter, tc, 12);
    }
    CHECK(!sawF);                 // never sends counter 0xF (reference wraps at 0xF)
    CHECK_EQ(maxCounter, 14);
    CHECK_EQ(tc, 4);              // ramps to 4 and stays

    // One full sweep of 12 slots advances the counter by one
    slot = 0; counter = 3; tc = 0;
    for (int i = 0; i < 12; i++) phevAdvance(slot, counter, tc, 12);
    CHECK_EQ(slot, 0);
    CHECK_EQ(counter, 4);
    CHECK_EQ(tc, 1);
    // Wrap: 14 -> 0
    slot = 11; counter = 14;
    phevAdvance(slot, counter, tc, 12);
    CHECK_EQ(counter, 0);
}

// ---------------------------------------------------------------------------
static void testIdManagement()
{
    uint8_t buf[8];
    phevBuildResetId(7, buf);
    const uint8_t expReset[8] = { 0xA1, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    CHECK(memcmp(buf, expReset, 8) == 0);

    phevBuildFindUnassigned(buf);
    const uint8_t expFind[8] = { 0x37, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    CHECK(memcmp(buf, expFind, 8) == 0);

    // assignID() byte layout (BMWPhevBMS.ino:3722-3760)
    const uint8_t dmc[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
    uint8_t f[4][8];
    phevBuildAssign(dmc, 3, f);
    const uint8_t e0[8] = { 0x12, 0xAB, 0x11, 0x22, 0x33, 0x44, 0xFF, 0xFF };
    const uint8_t e1[8] = { 0x12, 0xBA, 0x55, 0x66, 0x77, 0x88, 0xFF, 0xFF };
    const uint8_t e2[8] = { 0x5B, 0x03, 0x55, 0x66, 0x77, 0x88, 0xFF, 0xFF };
    const uint8_t e3[8] = { 0x37, 0x03, 0x55, 0x66, 0x77, 0x88, 0xFF, 0xFF };
    CHECK(memcmp(f[0], e0, 8) == 0);
    CHECK(memcmp(f[1], e1, 8) == 0);
    CHECK(memcmp(f[2], e2, 8) == 0);
    CHECK(memcmp(f[3], e3, 8) == 0);
}

// ---------------------------------------------------------------------------
static void testClassify()
{
    int m, s;
    CHECK_EQ(phevClassify(0x0F0, false, m, s), PHEV_RX_UNASSIGNED);
    CHECK_EQ(phevClassify(0x0F0, true,  m, s), PHEV_RX_NONE);    // extended: not ours
    // v7-beta7 decoded these as status; the CSCs never send them
    CHECK_EQ(phevClassify(0x0A0, false, m, s), PHEV_RX_NONE);
    CHECK_EQ(phevClassify(0x0AB, false, m, s), PHEV_RX_NONE);

    CHECK_EQ(phevClassify(0x100, false, m, s), PHEV_RX_STATUS);  CHECK_EQ(m, 1);
    CHECK_EQ(phevClassify(0x10B, false, m, s), PHEV_RX_STATUS);  CHECK_EQ(m, 12);
    CHECK_EQ(phevClassify(0x10C, false, m, s), PHEV_RX_NONE);    // module 13: no slot
    CHECK_EQ(phevClassify(0x110, false, m, s), PHEV_RX_NONE);

    CHECK_EQ(phevClassify(0x120, false, m, s), PHEV_RX_CELLS);   CHECK_EQ(m, 1);  CHECK_EQ(s, 0);
    CHECK_EQ(phevClassify(0x135, false, m, s), PHEV_RX_CELLS);   CHECK_EQ(m, 6);  CHECK_EQ(s, 1);
    CHECK_EQ(phevClassify(0x17B, false, m, s), PHEV_RX_CELLS);   CHECK_EQ(m, 12); CHECK_EQ(s, 5);
    CHECK_EQ(phevClassify(0x180, false, m, s), PHEV_RX_TEMPS);   CHECK_EQ(m, 1);
    CHECK_EQ(phevClassify(0x18B, false, m, s), PHEV_RX_TEMPS);   CHECK_EQ(m, 12);
    CHECK_EQ(phevClassify(0x190, false, m, s), PHEV_RX_NONE);
    CHECK_EQ(phevClassify(0x080, false, m, s), PHEV_RX_NONE);    // our own poll ID
}

// ---------------------------------------------------------------------------
static void testCellDecode()
{
    // 3700 mV = 0x0E74 -> lo 0x74, hi 0x0E; top two bits of hi are ignored
    uint8_t b[8] = { 0x74, 0x0E, 0x75, 0xCE, 0x00, 0x10, 0, 0 };
    uint16_t mv[3];
    CHECK_EQ(phevDecodeCells(0, b, mv), 3);
    CHECK_EQ(mv[0], 3700);
    CHECK_EQ(mv[1], 3701);          // 0xCE & 0x3F = 0x0E
    CHECK_EQ(mv[2], 0x1000);        // 4096
    CHECK_EQ(phevDecodeCells(5, b, mv), 1);   // sub-frame 5: cell 16 only
    CHECK_EQ(mv[0], 3700);

    // Presence needs sub-frames 0-2 only (8-cell modules)
    CHECK(!phevPresenceComplete(0x03));
    CHECK(phevPresenceComplete(0x07));
    CHECK(phevPresenceComplete(0x3F));
    CHECK(!phevPresenceComplete(0x38));
}

int main()
{
    testCrc();
    testPollFrames();
    testSequencer();
    testIdManagement();
    testClassify();
    testCellDecode();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
