#pragma once
// =============================================================================
// SocCalc.h - state of charge and pack voltage limits, per cell
//
// SoC is a linear interpolation of the average series-cell voltage
// (pack voltage / cells in series) between SOCLO and SOCHI, both in V/cell.
// Working per cell (not per module) handles packs that mix module sizes,
// e.g. 16 x 5S + 4 x 4S = 96S.
//
// Up to v7-beta7 SOCLO/SOCHI were V/module (default 18.0/25.2 = 6S x 3.0/4.2)
// and 0x351 sent the per-cell OV/UV limits x modules in series, i.e. 6x too
// low for a 6S Tesla pack. socMigrateToPerCell() converts saved settings.
//
// Pure functions: no Arduino calls, so they can be unit-tested on the host
// (test/soc_calc_test.cpp).
// =============================================================================

#define SOC_CELL_V_MIN   2.0f   // valid range for SOCLO/SOCHI in V/cell
#define SOC_CELL_V_MAX   4.5f
#define SOC_LEGACY_MIN   5.0f   // anything above this was saved as V/module

// SoC % (0..100) from pack voltage and cells in series. 0 when unknown.
static inline float socPercent(float packV, int seriesCells, float lo, float hi)
{
    if (seriesCells <= 0 || hi - lo < 0.01f) return 0.0f;
    float v = (packV / (float)seriesCells - lo) * 100.0f / (hi - lo);
    if (v > 100.0f) v = 100.0f;
    if (v < 0.0f)   v = 0.0f;
    return v;
}

// Pack voltage limit in 0.1 V units (0x351 encoding), from a per-cell limit.
static inline int packLimitDeciV(float cellV, int seriesCells)
{
    long v = (long)(cellV * (float)seriesCells * 10.0f + 0.5f);
    if (v < 0)     v = 0;
    if (v > 32767) v = 32767;
    return (int)v;
}

// Cells in series from the cells counted in all present modules and the
// number of parallel strings (rounded, so a missing module in one string
// does not drop the count by a whole string's worth of cells).
static inline int seriesCellsFrom(int totalCells, int parallel)
{
    if (totalCells <= 0) return 0;
    if (parallel <= 1)   return totalCells;
    return (totalCells + parallel / 2) / parallel;
}

// Convert saved SOCLO/SOCHI from V/module (v7-beta7 and earlier) to V/cell.
// Returns true if anything changed. The untouched defaults (18.0/25.2, which
// assumed 6S) become 3.0/4.2; other values are divided by cellsPerModule,
// the average over the configured modules (4.8 for 16 x 5S + 4 x 4S).
// Values that still fall outside the per-cell range are reset to 3.0/4.2.
static inline bool socMigrateToPerCell(float *lo, float *hi, float cellsPerModule)
{
    if (*lo <= SOC_LEGACY_MIN && *hi <= SOC_LEGACY_MIN) return false;
    bool wasDefault = (*lo > 17.95f && *lo < 18.05f && *hi > 25.15f && *hi < 25.25f);
    if (wasDefault || cellsPerModule < 1.0f) {
        *lo = 3.0f; *hi = 4.2f;
    } else {
        *lo /= cellsPerModule;
        *hi /= cellsPerModule;
    }
    if (*lo < SOC_CELL_V_MIN || *hi > SOC_CELL_V_MAX || *hi - *lo < 0.05f) {
        *lo = 3.0f; *hi = 4.2f;
    }
    return true;
}
