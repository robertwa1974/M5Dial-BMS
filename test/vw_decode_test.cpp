// =============================================================================
// vw_decode_test.cpp - host-side unit test for the VW CMU decoders (VWProtocol.h)
//
// Build and run from the repo root with any C++11 host compiler, e.g.
//   g++ -std=c++11 -Wall -Wextra -Isrc -Itest/stubs test/vw_decode_test.cpp -o vw_decode_test
//   ./vw_decode_test
//
// Expected IDs come from test/vw_reference_tables.h, which is generated from
// Tom de Bree's VWBMSV2 source by tools/gen_vw_reference_tables.py.
// Frames are packed here independently of the decoder, from the formulas in
// the reference (BMSModule.cpp:99-240), then decoded and compared.
// =============================================================================
#include <stdio.h>
#include <math.h>
#include "VWProtocol.h"
#include "vw_reference_tables.h"

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

#define CHECK_NEAR(a, b) do { \
    double _a = (a), _b = (b); \
    if (fabs(_a - _b) < 1e-4) { g_pass++; } \
    else { g_fail++; printf("FAIL %s:%d  %s ~= %s  (%f vs %f)\n", __FILE__, __LINE__, #a, #b, _a, _b); } \
} while (0)

// Independent packer: inverse of the reference decode for one sub-frame.
static void packSub(int sub, const uint16_t *mv, uint8_t *b)
{
    memset(b, 0, 8);
    uint16_t r0 = (uint16_t)(mv[0] - 1000);
    b[1] |= (uint8_t)((r0 & 0x0F) << 4);
    b[2]  = (uint8_t)(r0 >> 4);
    if (sub == 3) return;
    uint16_t r1 = (uint16_t)(mv[1] - 1000);
    uint16_t r2 = (uint16_t)(mv[2] - 1000);
    uint16_t r3 = (uint16_t)(mv[3] - 1000);
    b[3]  = (uint8_t)(r1 & 0xFF);
    b[4] |= (uint8_t)((r1 >> 8) & 0x0F);
    b[4] |= (uint8_t)((r2 & 0x0F) << 4);
    b[5]  = (uint8_t)(r2 >> 4);
    b[6]  = (uint8_t)(r3 & 0xFF);
    b[7] |= (uint8_t)((r3 >> 8) & 0x0F);
}

// ---------------------------------------------------------------------------
static void testCellIdMap()
{
    // Every ID in a wide window: formula must agree exactly with the
    // reference case table (present <-> mapped, same module and sub-frame).
    for (uint32_t id = 0x100; id <= 0x2FF; id++) {
        const VwRefCellId *ref = nullptr;
        for (size_t i = 0; i < sizeof(VW_REF_CELL_IDS) / sizeof(VW_REF_CELL_IDS[0]); i++)
            if (VW_REF_CELL_IDS[i].id == id) ref = &VW_REF_CELL_IDS[i];
        int m = -1, s = -1;
        bool ok = vwCellIdToModule(id, m, s);
        if (ref) {
            CHECK(ok);
            CHECK_EQ(m, ref->module);
            CHECK_EQ(s, ref->sub);
        } else {
            if (ok) printf("  unexpected mapping for 0x%03X\n", (unsigned)id);
            CHECK(!ok);
        }
    }
    // 0x1CC is module 8 sub-frame 0, not a base address (VWBMSV2.ino:191).
    int m, s;
    CHECK(vwCellIdToModule(0x1CC, m, s));
    CHECK_EQ(m, 8);
    CHECK_EQ(s, 0);
}

// ---------------------------------------------------------------------------
static void testCellRoundTrip()
{
    // 3700 mV -> raw 2700 (0xA8C), from the brief.
    {
        uint16_t in[4] = { 3700, 3700, 3700, 3700 }, out[4];
        uint8_t b[8];
        packSub(0, in, b);
        CHECK_EQ(((b[1] >> 4) | (b[2] << 4)), 0xA8C);
        CHECK_EQ(vwDecodeCells(0, b, out), 4);
        for (int i = 0; i < 4; i++) CHECK_EQ(out[i], 3700);
    }

    // All 13 cells of a standard module and 12 of an extender module,
    // each cell different so a swapped position would be caught.
    for (int modType = 0; modType < 2; modType++) {
        int nSub = (modType == 0) ? 4 : 3;
        int nCells = (modType == 0) ? 13 : 12;
        uint16_t cells[16];
        for (int c = 0; c < 16; c++) cells[c] = (uint16_t)(3001 + c * 61 + modType * 7);
        uint16_t got[16] = {0};
        for (int sub = 0; sub < nSub; sub++) {
            uint8_t b[8];
            packSub(sub, &cells[sub * 4], b);
            CHECK(vwCellFrameValid(sub, b, 8));
            uint16_t mv[4];
            int n = vwDecodeCells(sub, b, mv);
            CHECK_EQ(n, (sub == 3) ? 1 : 4);
            for (int k = 0; k < n; k++) got[sub * 4 + k] = mv[k];
        }
        for (int c = 0; c < nCells; c++) CHECK_EQ(got[c], cells[c]);
    }

    // Range limits: raw 0 -> 1000 mV, raw 0xFFE (sub 1, no guard byte hit) -> 5094 mV.
    {
        uint16_t in[4] = { 1000, 1000, 1000, 1000 }, out[4];
        uint8_t b[8];
        packSub(1, in, b);
        vwDecodeCells(1, b, out);
        for (int i = 0; i < 4; i++) CHECK_EQ(out[i], 1000);
    }
    {
        uint16_t in[4] = { 3000, 5094, 3000, 5094 }, out[4];
        uint8_t b[8];
        packSub(2, in, b);
        vwDecodeCells(2, b, out);
        CHECK_EQ(out[1], 5094);
        CHECK_EQ(out[3], 5094);
    }
}

// ---------------------------------------------------------------------------
static void testCellGuards()
{
    uint16_t in[4] = { 3700, 3710, 3720, 3730 };
    uint8_t b[8];

    for (int sub = 0; sub < 2; sub++) {
        packSub(sub, in, b);
        CHECK(vwCellFrameValid(sub, b, 8));
        uint8_t t[8];
        memcpy(t, b, 8); t[2] = 0xFF; CHECK(!vwCellFrameValid(sub, t, 8));
        memcpy(t, b, 8); t[5] = 0xFF; CHECK(!vwCellFrameValid(sub, t, 8));
        memcpy(t, b, 8); t[7] = 0xFF; CHECK(!vwCellFrameValid(sub, t, 8));
        CHECK(!vwCellFrameValid(sub, b, 7));          // short frame
    }

    // Sub-frame 2: only buf[2] is guarded.
    packSub(2, in, b);
    uint8_t t[8];
    memcpy(t, b, 8); t[5] = 0xFF; CHECK(vwCellFrameValid(2, t, 8));
    memcpy(t, b, 8); t[7] = 0xFF; CHECK(vwCellFrameValid(2, t, 8));
    memcpy(t, b, 8); t[2] = 0xFF; CHECK(!vwCellFrameValid(2, t, 8));

    // Sub-frame 3: only buf[2] guarded; only 3 bytes needed.
    packSub(3, in, b);
    CHECK(vwCellFrameValid(3, b, 3));
    CHECK(!vwCellFrameValid(3, b, 2));
    memcpy(t, b, 8); t[2] = 0xFF; CHECK(!vwCellFrameValid(3, t, 8));
}

// ---------------------------------------------------------------------------
static void testPresenceAndCellCount()
{
    // Presence needs sub-frames 0 and 1 (cells 1-8) only.
    CHECK(!vwPresenceComplete(0x00));
    CHECK(!vwPresenceComplete(0x01));
    CHECK(!vwPresenceComplete(0x02));
    CHECK(vwPresenceComplete(0x03));    // 8s module: no sub-frame 2 needed
    CHECK(vwPresenceComplete(0x07));
    CHECK(!vwPresenceComplete(0x0C));   // sub-frames 2-3 alone are not enough

    // Cell count from the highest sub-frame ever seen.
    CHECK_EQ(vwCellsFromSubFrames(0x03), 8);
    CHECK_EQ(vwCellsFromSubFrames(0x07), 12);
    CHECK_EQ(vwCellsFromSubFrames(0x0F), 13);
    CHECK_EQ(vwCellsFromSubFrames(0x0B), 13);   // sub 3 without sub 2 still means 13
    CHECK_EQ(vwCellsFromSubFrames(0x00), 8);
}

// ---------------------------------------------------------------------------
static void testTemperatures()
{
    VwTempResult r;

    // Type 1, normal branch with balance status.
    {
        uint8_t b[8] = { 0x80, 0x00, 0x34, 0x12, 0x84, 0x88, 0x00, 0x00 };
        CHECK(vwDecodeTemp1(b, 8, r));
        CHECK_NEAR(r.t[0], 0x80 * 0.5 - 43);   // 21.0
        CHECK_NEAR(r.t[1], 0x84 * 0.5 - 43);   // 23.0
        CHECK_NEAR(r.t[2], 0x88 * 0.5 - 43);   // 25.0
        CHECK_EQ(r.state[0], VW_T_VALID);
        CHECK_EQ(r.state[1], VW_T_VALID);
        CHECK_EQ(r.state[2], VW_T_VALID);
        CHECK(r.hasBalstat);
        CHECK_EQ(r.balstat, 0x1234);
    }
    // Type 1, buf[0] >= 0xDF: T0 from buf[3], no balstat; T1/T2 absent.
    {
        uint8_t b[8] = { 0xDF, 0x00, 0x00, 0x7C, 0xF0, 0xFF, 0x00, 0x00 };
        CHECK(vwDecodeTemp1(b, 8, r));
        CHECK_NEAR(r.t[0], 0x7C * 0.5 - 43);
        CHECK(!r.hasBalstat);
        CHECK_EQ(r.state[1], VW_T_ABSENT);
        CHECK_EQ(r.state[2], VW_T_ABSENT);
        CHECK_NEAR(r.t[1], VW_TEMP_INVALID);
    }
    // Type 1, buf[7] == 0xFD: T0 = buf[2]*0.5 - 40, others unchanged.
    {
        uint8_t b[8] = { 0x00, 0x00, 0x82, 0x00, 0x00, 0x00, 0x00, 0xFD };
        CHECK(vwDecodeTemp1(b, 8, r));
        CHECK_NEAR(r.t[0], 0x82 * 0.5 - 40);   // 25.0
        CHECK_EQ(r.state[0], VW_T_VALID);
        CHECK_EQ(r.state[1], VW_T_UNCHANGED);
        CHECK_EQ(r.state[2], VW_T_UNCHANGED);
        b[2] = 0xFD;
        CHECK(vwDecodeTemp1(b, 8, r));
        CHECK_EQ(r.state[0], VW_T_UNCHANGED);
    }
    CHECK(!vwDecodeTemp1((const uint8_t *)"\0\0\0\0\0\0\0", 7, r));

    // Type 2 (MEB): raw 0x82 split across buf[4] high nibble and buf[5] low nibble.
    {
        uint8_t b[8] = { 0, 0, 0, 0, 0x20, 0x08, 0, 0 };   // raw = (0x8 << 4) | 0x2 = 0x82
        CHECK(vwDecodeTemp2(b, 8, r));
        CHECK_NEAR(r.t[0], 0x82 * 0.5 - 40);   // 25.0
        CHECK_EQ(r.state[0], VW_T_VALID);
        CHECK_EQ(r.state[1], VW_T_UNCHANGED);
        b[5] = 0xDF;                            // initialising / spoof
        CHECK(!vwDecodeTemp2(b, 8, r));
    }
}

// ---------------------------------------------------------------------------
static void testModuleNumbers()
{
    // Type 1: low byte 1..10 used directly; 11..59 -> (low nibble)/2 + 1;
    // 60..63 fall through as 60..63 in the reference -> rejected here.
    CHECK_EQ(vwTemp1Module(0x1A555401UL), 1);
    CHECK_EQ(vwTemp1Module(0x1A55540AUL), 10);
    CHECK_EQ(vwTemp1Module(0x1A55540BUL), (0x0B / 2) + 1);   // 11 -> 6
    CHECK_EQ(vwTemp1Module(0x1A555410UL), 1);                 // 16 -> 0/2+1
    CHECK_EQ(vwTemp1Module(0x1A55541FUL), 8);                 // 31 -> 15/2+1
    CHECK_EQ(vwTemp1Module(0x1A55543BUL), (0x0B / 2) + 1);   // 59
    CHECK_EQ(vwTemp1Module(0x1A55543CUL), 0);                 // 60: out of range
    CHECK_EQ(vwTemp1Module(0x1A55543FUL), 0);                 // 63: out of range
    CHECK_EQ(vwTemp1Module(0x1A555400UL), 0);                 // 0

    // Type 2: (id & 0x0F) + 1
    CHECK_EQ(vwTemp2Module(VW_TEMP2_ID_MIN), 1);
    CHECK_EQ(vwTemp2Module(VW_TEMP2_ID_MAX), 15);
    CHECK_EQ(vwTemp2Module(0x1A5555FFUL), 16);   // caller's ID range excludes 0x...FF

    // Balance status: id & 0x0F, 1..14 only
    CHECK_EQ(vwBalstatModule(0x16A95470UL), 0);
    CHECK_EQ(vwBalstatModule(VW_BALSTAT_ID_MIN), 1);
    CHECK_EQ(vwBalstatModule(0x16A9547EUL), 14);
    CHECK_EQ(vwBalstatModule(VW_BALSTAT_ID_MAX), 0);   // 15 rejected (reference: CMU < 15)
}

// ---------------------------------------------------------------------------
static void testBalstat()
{
    uint8_t b[8] = { 0x00, 0xA0, 0xCB, 0x0D, 0, 0, 0, 0 };
    uint32_t bs = 0;
    CHECK(vwDecodeBalstat(b, 8, bs));
    CHECK_EQ(bs, (0xA0 >> 4) | (0xCB << 4) | (0x0D << 12));   // 0xDCBA
    CHECK_EQ(bs, 0xDCBA);
    // All of buf[3] kept (reference balstat is uint32_t)
    b[3] = 0xFF;
    vwDecodeBalstat(b, 8, bs);
    CHECK_EQ(bs, 0xFFCBAUL);
    CHECK(!vwDecodeBalstat(b, 3, bs));
}

// ---------------------------------------------------------------------------
static void testBalanceCommands()
{
    for (int i = 0; i < 12; i++) {
        CHECK_EQ(vwBalCmdIdA(VW_REF_BAL_IDS[i].module), VW_REF_BAL_IDS[i].a);
        CHECK_EQ(vwBalCmdIdB(VW_REF_BAL_IDS[i].module), VW_REF_BAL_IDS[i].b);
    }
    CHECK_EQ(vwBalCmdIdA(0), 0);
    CHECK_EQ(vwBalCmdIdA(13), 0);
    CHECK_EQ(vwBalCmdIdB(16), 0);

    // Cells 0, 7, 8 and 12 bleed.
    uint8_t a[8], b[8];
    vwBuildBalFrames((1u << 0) | (1u << 7) | (1u << 8) | (1u << 12), a, b);
    const uint8_t expA[8] = { 0x08, 0, 0, 0, 0, 0, 0, 0x08 };
    const uint8_t expB[8] = { 0x08, 0, 0, 0, 0x08, 0xFE, 0xFE, 0xFE };
    CHECK(memcmp(a, expA, 8) == 0);
    CHECK(memcmp(b, expB, 8) == 0);

    vwBuildBalStopFrames(a, b);
    const uint8_t stopA[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    const uint8_t stopB[8] = { 0, 0, 0, 0, 0xFE, 0xFE, 0xFE, 0xFE };
    CHECK(memcmp(a, stopA, 8) == 0);
    CHECK(memcmp(b, stopB, 8) == 0);

    // Poll payloads (VWBMSV2.ino:3104-3129)
    const uint8_t p2[8] = { 0x45, 0x01, 0x28, 0x00, 0x00, 0x00, 0x00, 0x30 };
    CHECK(memcmp(VW_POLL_FRAME_2, p2, 8) == 0);
    for (int i = 0; i < 8; i++) CHECK_EQ(VW_POLL_FRAME_1[i], 0);
}

// ---------------------------------------------------------------------------
static void testBalanceLogic()
{
    // Gate: absolute balanceVoltage AND hysteresis over the lowest cell.
    CHECK(vwBalanceGate(4.11f, 4.00f, 4.10f, 0.02f));
    CHECK(!vwBalanceGate(4.09f, 4.00f, 4.10f, 0.02f));   // below balanceVoltage
    CHECK(!vwBalanceGate(4.11f, 4.10f, 4.10f, 0.02f));   // spread under hysteresis

    float cells[13];
    for (int i = 0; i < 13; i++) cells[i] = 4.00f;
    cells[3] = 4.05f; cells[12] = 4.06f; cells[5] = 4.019f;
    CHECK_EQ(vwBalanceMask(cells, 13, 4.00f, 0.02f), (1u << 3) | (1u << 12));
    CHECK_EQ(vwBalanceMask(cells, 12, 4.00f, 0.02f), (1u << 3));   // cell 13 not counted

    // State machine: always-imbalanced pack. Expected actions per counter
    // value, following the reference's balcnt handling.
    uint8_t cnt = 0;
    int stops = 0, sends = 0;
    for (int tick = 0; tick <= 60; tick++) {   // counter values 0..60
        uint8_t before = cnt;
        VwBalAction act = vwBalanceAction(cnt);
        if (before == 1) CHECK_EQ(act, VW_BAL_STOP);
        if (before >= 2 && before <= 10) CHECK_EQ(act, VW_BAL_IDLE);
        if (before == 11 || before == 15 || before == 55) CHECK_EQ(act, VW_BAL_SEND);
        if (before == 12 || before == 56 || before == 60) CHECK_EQ(act, VW_BAL_IDLE);
        if (act == VW_BAL_STOP) stops++;
        if (act == VW_BAL_SEND) sends++;
        cnt++;
    }
    CHECK_EQ(stops, 1);
    CHECK_EQ(sends, 10);   // 11, 15, 20 ... 55
    // Counter 61 wraps to 0 (idle), then 1 is the next STOP.
    CHECK_EQ(cnt, 61);
    CHECK_EQ(vwBalanceAction(cnt), VW_BAL_IDLE);
    CHECK_EQ(cnt, 0);
}

// ---------------------------------------------------------------------------
static void testProtectionValues()
{
    // Not bleeding: live readings are un-bled and used as-is.
    CHECK_NEAR(vwProtectionHigh(4.15f, false, true, 4.30f, 100), 4.15f);

    // Bleeding: live + margin ...
    CHECK_NEAR(vwProtectionHigh(4.15f, true, false, 0.0f, 0), 4.15f + VW_BLEED_OV_MARGIN);
    // ... or a fresh, higher un-bled snapshot
    CHECK_NEAR(vwProtectionHigh(4.10f, true, true, 4.19f, VW_SNAP_MAX_AGE_MS), 4.19f);
    // ... but a snapshot older than the cap is ignored
    CHECK_NEAR(vwProtectionHigh(4.10f, true, true, 4.19f, VW_SNAP_MAX_AGE_MS + 1),
               4.10f + VW_BLEED_OV_MARGIN);
    // ... and a lower snapshot never reduces the value
    CHECK_NEAR(vwProtectionHigh(4.15f, true, true, 4.00f, 0), 4.15f + VW_BLEED_OV_MARGIN);

    // Stop-at-margin: with OVERV 4.20, a bled live reading of 4.16 trips
    // (4.16 + 0.05 > 4.20); 4.14 does not.
    CHECK(vwProtectionHigh(4.16f, true, false, 0.0f, 0) > 4.20f);
    CHECK(!(vwProtectionHigh(4.14f, true, false, 0.0f, 0) > 4.20f));

    // Pack low leaves out bleeding cells and ignored cells.
    float cv[13];
    for (int i = 0; i < 13; i++) cv[i] = 4.10f;
    cv[2] = 3.95f;    // bleeding, reads low - excluded
    cv[7] = 4.02f;    // not bleeding - real minimum
    cv[11] = 0.10f;   // unpopulated input - below ignore threshold
    CHECK_NEAR(vwLowCellExcluding(cv, 13, (uint16_t)(1u << 2), 0.5f, 5.0f), 4.02f);
    CHECK_NEAR(vwLowCellExcluding(cv, 13, 0, 0.5f, 5.0f), 3.95f);
    CHECK_NEAR(vwLowCellExcluding(cv, 13, 0, 0.5f, 3.90f), 3.90f);   // keeps a lower running min
}

int main()
{
    testCellIdMap();
    testCellRoundTrip();
    testCellGuards();
    testPresenceAndCellCount();
    testTemperatures();
    testModuleNumbers();
    testBalstat();
    testBalanceCommands();
    testBalanceLogic();
    testProtectionValues();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
