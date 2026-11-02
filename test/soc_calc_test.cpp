// =============================================================================
// soc_calc_test.cpp - host-side unit test for src/SocCalc.h
//
//   g++ -std=c++11 -Wall -Wextra -Isrc test/soc_calc_test.cpp -o soc_calc_test
//   ./soc_calc_test
// =============================================================================
#include <stdio.h>
#include <math.h>
#include "SocCalc.h"

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
    double _a = (double)(a), _b = (double)(b); \
    if (fabs(_a - _b) <= (tol)) { g_pass++; } \
    else { g_fail++; printf("FAIL %s:%d  %s ~= %s  (%f vs %f)\n", __FILE__, __LINE__, #a, #b, _a, _b); } \
} while (0)

int main()
{
    // --- socPercent -------------------------------------------------------
    // Mixed pack: 16 x 5S + 4 x 4S = 96S at 363.18 V -> 3.783 V/cell
    CHECK_NEAR(socPercent(363.18f, 96, 3.0f, 4.2f), (363.18 / 96 - 3.0) / 1.2 * 100, 0.01);
    // What v7-beta7 showed for that pack: 363.18 / 20 modules vs 18.0..25.2
    CHECK_NEAR((363.18 / 20 - 18.0) / 7.2 * 100, 2.2, 0.1);
    // Endpoints and clamping
    CHECK_NEAR(socPercent(3.0f * 96, 96, 3.0f, 4.2f), 0.0, 0.01);
    CHECK_NEAR(socPercent(4.2f * 96, 96, 3.0f, 4.2f), 100.0, 0.01);
    CHECK_NEAR(socPercent(2.5f * 96, 96, 3.0f, 4.2f), 0.0, 0.0);
    CHECK_NEAR(socPercent(4.5f * 96, 96, 3.0f, 4.2f), 100.0, 0.0);
    // Unknown cell count or bad range -> 0, never divide by zero
    CHECK_NEAR(socPercent(363.0f, 0, 3.0f, 4.2f), 0.0, 0.0);
    CHECK_NEAR(socPercent(363.0f, 96, 4.2f, 4.2f), 0.0, 0.0);
    CHECK_NEAR(socPercent(363.0f, 96, 4.2f, 3.0f), 0.0, 0.0);

    // --- packLimitDeciV (0x351) -------------------------------------------
    CHECK(packLimitDeciV(4.2f, 96) == 4032);    // 403.2 V (was 84.0 V: 4.2 x 20)
    CHECK(packLimitDeciV(3.0f, 96) == 2880);    // 288.0 V (was 60.0 V)
    CHECK(packLimitDeciV(4.2f, 12) == 504);     // 2 x 6S Tesla: 50.4 V (was 8.4 V)
    CHECK(packLimitDeciV(4.2f, 0) == 0);
    CHECK(packLimitDeciV(4.2f, 10000) == 32767);  // clamp to int16

    // --- seriesCellsFrom --------------------------------------------------
    CHECK(seriesCellsFrom(96, 1) == 96);
    CHECK(seriesCellsFrom(96, 0) == 96);
    CHECK(seriesCellsFrom(192, 2) == 96);
    CHECK(seriesCellsFrom(187, 2) == 94);       // one 5S module missing: rounds
    CHECK(seriesCellsFrom(0, 2) == 0);

    // --- socMigrateToPerCell ----------------------------------------------
    float lo, hi;
    // Untouched v7-beta7 defaults -> 3.0/4.2 whatever the cell count
    lo = 18.0f; hi = 25.2f;
    CHECK(socMigrateToPerCell(&lo, &hi, 5));
    CHECK_NEAR(lo, 3.0, 1e-6); CHECK_NEAR(hi, 4.2, 1e-6);
    // User-calibrated per-module values are divided by cells per module
    lo = 19.2f; hi = 24.6f;     // 6S, 3.2..4.1
    CHECK(socMigrateToPerCell(&lo, &hi, 6));
    CHECK_NEAR(lo, 3.2, 1e-5); CHECK_NEAR(hi, 4.1, 1e-5);
    lo = 14.4f; hi = 20.16f;    // the 4.8-cell workaround for 96S / 20 modules
    CHECK(socMigrateToPerCell(&lo, &hi, 4.8f));  // average of 16 x 5 + 4 x 4
    CHECK_NEAR(lo, 3.0, 1e-5); CHECK_NEAR(hi, 4.2, 1e-5);
    // Already per cell -> unchanged, returns false
    lo = 3.1f; hi = 4.15f;
    CHECK(!socMigrateToPerCell(&lo, &hi, 6));
    CHECK_NEAR(lo, 3.1, 1e-6); CHECK_NEAR(hi, 4.15, 1e-6);
    // Results outside the per-cell range fall back to 3.0/4.2
    lo = 10.0f; hi = 30.0f;     // /6 -> 1.67..5.0
    CHECK(socMigrateToPerCell(&lo, &hi, 6));
    CHECK_NEAR(lo, 3.0, 1e-6); CHECK_NEAR(hi, 4.2, 1e-6);
    lo = 18.0f; hi = 25.0f;     // unknown cells per module
    CHECK(socMigrateToPerCell(&lo, &hi, 0));
    CHECK_NEAR(lo, 3.0, 1e-6); CHECK_NEAR(hi, 4.2, 1e-6);

    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
