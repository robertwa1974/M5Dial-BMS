#pragma once
// =============================================================================
// BMSModule.h - Tesla BMS module data model
// Ported for M5Dial ESP32-S3:
//   - #include "config.h" -> #include "bms_config.h"
//   - No other changes: pure data model, zero hardware dependencies
// =============================================================================
#include <stdint.h>
#include "bms_config.h"

class BMSModule
{
public:
    BMSModule();
    void readStatus();
    bool readModuleValues();

    // Voltage getters
    float getCellVoltage(int cell);
    float getLowCellV();
    float getHighCellV();
    float getAverageV();
    float getModuleVoltage();
    float getHighestModuleVolt();
    float getLowestModuleVolt();
    float getHighestCellVolt(int cell);
    float getLowestCellVolt(int cell);

    // Temperature getters
    float getTemperature(int temp);
    float getLowTemp();
    float getHighTemp();
    float getAvgTemp();
    float getHighestTemp();
    float getLowestTemp();

    // Status/fault getters
    uint8_t getFaults();
    uint8_t getAlerts();
    uint8_t getCOVCells();
    uint8_t getCUVCells();

    // Address and existence
    void setAddress(int newAddr);
    int  getAddress();
    bool isExisting();
    void setExists(bool ex);

    // Configuration setters
    void settempsensor(int tempsensor);
    void setIgnoreCell(float Ignore);
    void setNumCells(int n);
    int  getNumCells() const;
    // Direct write — used by CAN-sourced slave types (BMW i3)
    void setCellVoltage(int cell, float v);
    void setTemperature(int idx, float t);
    void setModuleVoltage(float v);
    void setFaults(uint8_t f);
    void setAlerts(uint8_t a);

private:
    float   cellVolt[16];           // up to 16 cells (6=Tesla, 12=BMW i3, 16=BMW PHEV)
    float   lowestCellVolt[16];
    float   highestCellVolt[16];
    float   moduleVolt;
    float   temperatures[2];
    float   lowestTemperature;
    float   highestTemperature;
    float   lowestModuleVolt;
    float   highestModuleVolt;
    float   IgnoreCell;         // cells below this voltage are ignored in low-cell calc
    bool    exists;
    int     alerts;
    int     faults;
    int     COVFaults;
    int     CUVFaults;
    int     sensor;             // 0=avg both, 1=TS1 only, 2=TS2 only
    uint8_t moduleAddress;      // 1 to MAX_MODULE_ADDR (0x3E)
    uint8_t numCells;           // active cell count for this module (1-16)
};
