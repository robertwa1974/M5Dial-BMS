#pragma once
// =============================================================================
// BMSModuleManager.h
// v5: added setBalanceInhibit(), sendCANFrames(), auto-balance scheduling
// =============================================================================
#include "bms_config.h"
#include "BMSModule.h"
#include "CANManager.h"

class BMSModuleManager
{
public:
    BMSModuleManager();

    // Board enumeration
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();

    // Runtime operations
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void balanceCells();
    // VW CMU: send stop frames if a bleed is active and reset the duty cycle
    void stopBalancingVW();

    // Configuration
    void readSetpoints();
    void setBatteryID(int id);
    void setPstrings(int Pstrings);
    void setSensors(int sensor, float Ignore);

    // v5: balancing inhibit (drive mode)
    void setBalanceInhibit(bool inhibit);
    bool getBalanceInhibit();

    // v5: enable/disable auto-balancing
    void setAutoBalance(bool enabled);
    bool getAutoBalance();

    // Pack-level data
    float getPackVoltage();
    float getAvgTemperature();
    float getHighTemperature();
    float getLowTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    // Pack high cell for overvoltage checks (0x35A alarm, 0x373 max cell).
    // Same as getHighCellVolt() except in VW CMU mode while cells bleed.
    float getProtectionHighCellVolt();
    float getLowVoltage();
    float getHighVoltage();
    int   getNumModules();
    bool  isFaultedState();
    // Returns the active cell count for a specific module address.
    int   getModuleCells(int addr);
    // BMW i3 CAN-sourced data path (called instead of UART getAllVoltTemp)
    void  getAllVoltTempFromCAN();
    // BMW PHEV CAN-sourced data path
    void  getAllVoltTempFromPHEV();
    // VW CMU (MEB / MQB) CAN-sourced data path
    void  getAllVoltTempFromVW();
    // Chrysler Pacifica BPCM CAN-sourced data path
    void  getAllVoltTempFromPacifica();

    // Per-module accessor
    BMSModule& getModule(int addr);

    // Serial console debug
    void printPackSummary();
    void printPackDetails();

private:
    float     packVolt;
    int       Pstring;
    float     LowCellVolt;
    float     HighCellVolt;
    float     lowestPackVolt;
    float     highestPackVolt;
    float     lowestPackTemp;
    float     highestPackTemp;
    BMSModule modules[MAX_MODULE_ADDR + 1];
    int       batteryID;
    int       numFoundModules;
    bool      isFaulted;
    bool      balanceInhibit;   // true = drive mode, skip balancing
    bool      autoBalance;      // true = balance automatically each cycle
    // VW CMU: sub-frames each module has ever sent (cell count, see VWProtocol.h)
    uint8_t   vwSubSeen[MAX_MODULE_ADDR + 1];
    // VW CMU balancing state (reference balcnt duty cycle)
    uint8_t   vwBalCnt;
    bool      vwBalCmdSent;     // balance frames sent since the last stop
    bool      vwBleeding;       // a non-zero mask is active (cells read low)
    uint16_t  vwBleedMask[VW_MAX_MODS + 1];   // cells currently bleeding
    uint32_t  vwLastStopMs;     // when stop frames were last requested
    // Un-bled snapshot: pack high taken while no cell was bleeding and
    // VW_SETTLE_MS after the last stop (see VWProtocol.h)
    float     vwSnapHigh;
    uint32_t  vwSnapMs;         // 0 = none yet
    void      balanceCellsVW();
};
