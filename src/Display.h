#pragma once
// =============================================================================
// Display.h - LVGL display manager for M5Dial 240x240 round screen
//
// Pages:
//   0 = Pack summary (voltage arc, SoC, cell stats, module count, WiFi IP)
//   1..N = Per-module detail (cell bars, temps, fault badge)
//   N+1 = Settings info page (OV/UV thresholds, balancing, CAN, WiFi status)
//
// Encoder integration:
//   Rotate = page up/down
//   Press  = return to page 0
// =============================================================================
#include <Arduino.h>

class Display {
public:
    Display();
    bool begin();                        // init M5GFX + LVGL
    void tick();                         // call every ~10ms from loop
    void setPage(int page);
    int  getPage();
    void markDirty();                    // force full redraw on next tick
    void showStartup(const char *msg);   // splash update before BMS ready

private:
    int  currentPage;
    bool dirty;
    bool initialized;
    uint32_t lastLvglTick;

    void buildPackPage();
    void buildModulePage(int addr);
    void buildSettingsPage();
    void updatePackPage();
    void updateModulePage(int addr);
    void updateSettingsPage();
};
