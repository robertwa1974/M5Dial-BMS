// =============================================================================
// pacifica_decode_test.cpp - host-side unit test for the Pacifica BPCM
// decoders (PacificaProtocol.h)
//
// Build and run from the repo root with any C++11 host compiler, e.g.
//   g++ -std=c++11 -Wall -Wextra -Isrc -Itest/stubs test/pacifica_decode_test.cpp -o pacifica_decode_test
//   ./pacifica_decode_test
//
// Expected layout is Slyd's posted DBC (see PacificaProtocol.h). Frames are
// packed here independently of the decoder: cells/temps byte by byte, and the
// 0x150 bit fields with a DBC Motorola bit walker (not the decoder's shift).
// =============================================================================
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "PacificaProtocol.h"

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

// DBC Motorola (@0) packer: walk from the start bit (MSB) through `length`
// bits in DBC sawtooth order - down within a byte, then to bit 7 of the next.
static void packMotorola(uint8_t *buf, int start, int length, uint32_t value)
{
    int bit = start;
    for (int i = length - 1; i >= 0; i--) {
        int byte = bit / 8, pos = bit % 8;
        if (value & (1UL << i)) buf[byte] |= (uint8_t)(1 << pos);
        else                    buf[byte] &= (uint8_t)~(1 << pos);
        bit = (pos == 0) ? (byte + 1) * 8 + 7 : bit - 1;
    }
}

static void testCellIds()
{
    // Slyd's DBC: 0x487..0x49E = M1 cells 1-4 ... M6 cells 93-96, in order
    int seen[PAC_MAX_MODS + 1][4];
    memset(seen, 0, sizeof(seen));
    for (uint32_t id = 0x487; id <= 0x49E; id++) {
        int m = -1, s = -1;
        CHECK(pacCellIdToModule(id, m, s));
        int firstCell = (int)(id - 0x487) * 4 + 1;          // 1-based pack cell
        CHECK_EQ(m, (firstCell - 1) / 16 + 1);
        CHECK_EQ(s * 4 + 1, (firstCell - 1) % 16 + 1);       // cell within module
        if (m >= 1 && m <= PAC_MAX_MODS && s >= 0 && s < 4) seen[m][s]++;
    }
    for (int m = 1; m <= PAC_MAX_MODS; m++)
        for (int s = 0; s < 4; s++) CHECK_EQ(seen[m][s], 1);
    int m, s;
    CHECK(!pacCellIdToModule(0x486, m, s));
    CHECK(!pacCellIdToModule(0x49F, m, s));
    CHECK(!pacCellIdToModule(0x4A0, m, s));
}

static void testCells()
{
    const uint16_t mv[4] = { 3800, 3801, 4199, 3300 };
    uint8_t b[8];
    for (int k = 0; k < 4; k++) { b[2 * k] = (uint8_t)(mv[k] >> 8); b[2 * k + 1] = (uint8_t)(mv[k] & 0xFF); }
    uint16_t out[4];
    CHECK_EQ(pacDecodeCells(b, 8, out), 4);
    for (int k = 0; k < 4; k++) CHECK_EQ(out[k], mv[k]);
    // Same values through the DBC signal definitions (7|16, 23|16, 39|16, 55|16)
    uint8_t d[8] = { 0 };
    packMotorola(d, 7, 16, 3800); packMotorola(d, 23, 16, 3801);
    packMotorola(d, 39, 16, 4199); packMotorola(d, 55, 16, 3300);
    CHECK(memcmp(d, b, 8) == 0);
    CHECK_EQ(pacDecodeCells(b, 7, out), 0);   // short frame rejected
}

static void testTemps()
{
    CHECK_EQ(pacTempIdFirstModule(0x4A0), 1);
    CHECK_EQ(pacTempIdFirstModule(0x4A1), 4);
    CHECK_EQ(pacTempIdFirstModule(0x4A2), 0);
    const uint8_t b[6] = { 60, 61, 40, 0, 255, 65 };   // 20, 21, 0, -40, 215, 25 C
    float t[6];
    CHECK(pacDecodeTemps(b, 6, t));
    CHECK_NEAR(t[0], 20.0); CHECK_NEAR(t[1], 21.0); CHECK_NEAR(t[2], 0.0);
    CHECK_NEAR(t[3], -40.0); CHECK_NEAR(t[4], 215.0); CHECK_NEAR(t[5], 25.0);
    CHECK(!pacDecodeTemps(b, 5, t));

    const uint8_t c[8] = { 65, 63, 0, 0, 0, 0, 0, 0 };  // outlet 25, inlet 23
    float out, in;
    CHECK(pacDecodeCoolant(c, 8, out, in));
    CHECK_NEAR(out, 25.0); CHECK_NEAR(in, 23.0);
}

static void testStatus()
{
    // Each field alone, then all together, so a misplaced bit shows up
    const uint16_t raws[] = { 0, 1, 1024, 1953, 2047 };   // 1953 x 0.192 = 375 V
    for (int hv = 0; hv <= 1; hv++)
        for (int ext = 0; ext <= 1; ext++)
            for (unsigned r = 0; r < sizeof(raws) / sizeof(raws[0]); r++) {
                uint8_t b[8] = { 0 };
                packMotorola(b, 43, 1, (uint32_t)hv);
                packMotorola(b, 30, 1, (uint32_t)ext);
                packMotorola(b, 26, 11, raws[r]);
                PacStatus s;
                CHECK(pacDecodeStatus(b, 8, s));
                CHECK_EQ(s.hvilClosed, hv);
                CHECK_EQ(s.hvilExtClosed, ext);
                CHECK_EQ(s.field26Raw, raws[r]);
            }
    // Fields must not overlap: all-ones elsewhere must not leak into them
    uint8_t b[8];
    memset(b, 0xFF, 8);
    packMotorola(b, 43, 1, 0);
    packMotorola(b, 30, 1, 0);
    packMotorola(b, 26, 11, 0);
    PacStatus s;
    CHECK(pacDecodeStatus(b, 8, s));
    CHECK(!s.hvilClosed);
    CHECK(!s.hvilExtClosed);
    CHECK_EQ(s.field26Raw, 0);
    CHECK(!pacDecodeStatus(b, 7, s));
}

int main()
{
    testCellIds();
    testCells();
    testTemps();
    testStatus();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
