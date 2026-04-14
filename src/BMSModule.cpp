// =============================================================================
// BMSModule.cpp - Tesla BMS module data model implementation
// Ported for M5Dial ESP32-S3:
//   - #include "config.h" -> #include "bms_config.h"
//   - All logic unchanged: Steinhart-Hart temps, voltage scaling, CRC reads
// =============================================================================
#include "bms_config.h"
#include "BMSModule.h"
#include "BMSUtil.h"
#include "Logger.h"

BMSModule::BMSModule()
{
    for (int i = 0; i < 16; i++)
    {
        cellVolt[i]        = 0.0f;
        lowestCellVolt[i]  = 5.0f;
        highestCellVolt[i] = 0.0f;
    }
    moduleVolt        = 0.0f;
    temperatures[0]   = 0.0f;
    temperatures[1]   = 0.0f;
    lowestTemperature  = 200.0f;
    highestTemperature = -100.0f;
    lowestModuleVolt   = 200.0f;
    highestModuleVolt  = 0.0f;
    IgnoreCell         = 0.5f;   // default: ignore cells below 0.5V (disconnected)
    exists             = false;
    moduleAddress      = 0;
    alerts             = 0;
    faults             = 0;
    COVFaults          = 0;
    CUVFaults          = 0;
    sensor             = 0;      // default: average both temperature sensors
    numCells           = 6;      // default; overridden by setNumCells()
}

// ---------------------------------------------------------------------------
// readStatus - read alert/fault registers (called inside readModuleValues)
// ---------------------------------------------------------------------------
void BMSModule::readStatus()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = moduleAddress << 1;
    payload[1] = REG_ALERT_STATUS;
    payload[2] = 0x04;  // read 4 bytes starting at REG_ALERT_STATUS
    BMSUtil::sendDataWithReply(payload, 3, false, buff, 7);
    alerts    = buff[3];
    faults    = buff[4];
    COVFaults = buff[5];
    CUVFaults = buff[6];
}

uint8_t BMSModule::getFaults()    { return faults;    }
uint8_t BMSModule::getAlerts()    { return alerts;    }
uint8_t BMSModule::getCOVCells()  { return COVFaults; }
uint8_t BMSModule::getCUVCells()  { return CUVFaults; }

// ---------------------------------------------------------------------------
// readModuleValues - full ADC read: module voltage, 6 cell voltages, 2 temps
//
// Wire protocol (unchanged):
//   1. Set ADC_CTRL: auto mode, all inputs
//   2. Set IO_CTRL:  enable temperature VSS pins
//   3. Trigger ADC_CONV
//   4. Read 18 bytes starting at REG_GPAI -> 22 byte response (18 data + addr/cmd/len/CRC)
//   5. Validate CRC and header bytes
//   6. Decode voltages with scaling constants from BQ76PL536 datasheet
//   7. Decode temperatures via Steinhart-Hart equation
// ---------------------------------------------------------------------------
bool BMSModule::readModuleValues()
{
    uint8_t payload[4];
    uint8_t buff[50];
    uint8_t calcCRC;
    bool    retVal = false;
    int     retLen;
    float   tempCalc, tempTemp;

    payload[0] = moduleAddress << 1;

    readStatus();
    Logger::debug("Module %i  alerts=%X  faults=%X  COV=%X  CUV=%X",
                  moduleAddress, alerts, faults, COVFaults, CUVFaults);

    // Configure ADC: auto mode, read all inputs (both temps + pack + 6 cells)
    payload[1] = REG_ADC_CTRL;
    payload[2] = 0b00111101;
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

    // Enable temperature measurement VSS pins
    payload[1] = REG_IO_CTRL;
    payload[2] = 0b00000011;
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

    // Start ADC conversion
    payload[1] = REG_ADC_CONV;
    payload[2] = 1;
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

    // Read 18 bytes: 2B moduleV + 12B cellV (6x2B) + 4B temp (2x2B)
    // Full response = 22 bytes: addr + cmd + len + 18 data + CRC
    payload[1] = REG_GPAI;
    payload[2] = 0x12;  // 18 bytes
    retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 22);

    calcCRC = BMSUtil::genCRC(buff, retLen - 1);
    Logger::debug("Sent CRC: %x     Calculated CRC: %x", buff[21], calcCRC);

    if ((retLen == 22) && (buff[21] == calcCRC))
    {
        if (buff[0] == (moduleAddress << 1) && buff[1] == REG_GPAI && buff[2] == 0x12)
        {
            // Module voltage: 16-bit ADC * 0.002034609 -> volts
            moduleVolt = (buff[3] * 256 + buff[4]) * 0.002034609f;
            if (moduleVolt > highestModuleVolt) highestModuleVolt = moduleVolt;
            if (moduleVolt < lowestModuleVolt)  lowestModuleVolt  = moduleVolt;

            // Cell voltages: 16-bit ADC * 0.000381493 -> volts
            for (int i = 0; i < numCells && i < 6; i++)
            {
                cellVolt[i] = (buff[5 + (i * 2)] * 256 + buff[6 + (i * 2)]) * 0.000381493f;
                if (lowestCellVolt[i]  > cellVolt[i] && cellVolt[i] >= IgnoreCell)
                    lowestCellVolt[i]  = cellVolt[i];
                if (highestCellVolt[i] < cellVolt[i])
                    highestCellVolt[i] = cellVolt[i];
            }

            // Temperature via Steinhart-Hart equation (both sensors)
            // Sensor 1 (negative terminal)
            tempTemp  = (1.78f / ((buff[17] * 256 + buff[18] + 2) / 33046.0f) - 3.57f);
            tempTemp *= 1000.0f;
            tempCalc  = 1.0f / (0.0007610373573f
                              + (0.0002728524832f  * logf(tempTemp))
                              + (powf(logf(tempTemp), 3) * 0.0000001022822735f));
            temperatures[0] = tempCalc - 273.15f;

            // Sensor 2 (positive terminal)
            tempTemp  = 1.78f / ((buff[19] * 256 + buff[20] + 9) / 33068.0f) - 3.57f;
            tempTemp *= 1000.0f;
            tempCalc  = 1.0f / (0.0007610373573f
                              + (0.0002728524832f  * logf(tempTemp))
                              + (powf(logf(tempTemp), 3) * 0.0000001022822735f));
            temperatures[1] = tempCalc - 273.15f;

            if (getLowTemp()  < lowestTemperature)  lowestTemperature  = getLowTemp();
            if (getHighTemp() > highestTemperature) highestTemperature = getHighTemp();

            Logger::debug("Got voltage and temperature readings");
            retVal = true;
        }
    }
    else
    {
        Logger::error("Invalid module response: module %i  len=%i  rxCRC=%i  calcCRC=%i",
                      moduleAddress, retLen, buff[21], calcCRC);
    }

    return retVal;
}

// ---------------------------------------------------------------------------
// Voltage getters
// ---------------------------------------------------------------------------
float BMSModule::getCellVoltage(int cell)
{
    if (cell < 0 || cell >= numCells) return 0.0f;
    return cellVolt[cell];
}

float BMSModule::getLowCellV()
{
    float lowVal = 10.0f;
    for (int i = 0; i < numCells; i++)
        if (cellVolt[i] < lowVal && cellVolt[i] > IgnoreCell) lowVal = cellVolt[i];
    return lowVal;
}

float BMSModule::getHighCellV()
{
    float hiVal = 0.0f;
    for (int i = 0; i < numCells; i++)
        if (cellVolt[i] > hiVal) hiVal = cellVolt[i];
    return hiVal;
}

float BMSModule::getAverageV()
{
    int   x      = 0;
    float avgVal = 0.0f;
    for (int i = 0; i < numCells; i++)
    {
        if (cellVolt[i] > IgnoreCell) { avgVal += cellVolt[i]; x++; }
    }
    if (x == 0) return 0.0f;
    return avgVal / x;
}

float BMSModule::getModuleVoltage()    { return moduleVolt;        }
float BMSModule::getHighestModuleVolt(){ return highestModuleVolt; }
float BMSModule::getLowestModuleVolt() { return lowestModuleVolt;  }

float BMSModule::getHighestCellVolt(int cell)
{
    if (cell < 0 || cell > 5) return 0.0f;
    return highestCellVolt[cell];
}

float BMSModule::getLowestCellVolt(int cell)
{
    if (cell < 0 || cell > 5) return 0.0f;
    return lowestCellVolt[cell];
}

// ---------------------------------------------------------------------------
// Temperature getters
// ---------------------------------------------------------------------------
float BMSModule::getTemperature(int temp)
{
    if (temp < 0 || temp > 1) return 0.0f;
    return temperatures[temp];
}

float BMSModule::getLowTemp()
{
    return (temperatures[0] < temperatures[1]) ? temperatures[0] : temperatures[1];
}

float BMSModule::getHighTemp()
{
    return (temperatures[0] < temperatures[1]) ? temperatures[1] : temperatures[0];
}

float BMSModule::getAvgTemp()
{
    // Filter sensors that are below the ignore threshold (disconnected thermistors
    // read ~ -89°C on the BQ76PL536A).  IgnoreTempThresh is typically -70°C.
    extern EEPROMSettings settings;
    float thresh = settings.IgnoreTempThresh;

    if (sensor != 0) {
        // Single-sensor mode: return that sensor, or 0 if below threshold
        float t = temperatures[sensor - 1];
        return (t > thresh) ? t : 0.0f;
    }

    // Both-sensor mode: average only valid sensors
    bool v0 = temperatures[0] > thresh;
    bool v1 = temperatures[1] > thresh;
    if (v0 && v1)  return (temperatures[0] + temperatures[1]) / 2.0f;
    if (v0)        return temperatures[0];
    if (v1)        return temperatures[1];
    return 0.0f;   // no valid sensor — caller's IgnoreTempThresh check will skip this module
}

float BMSModule::getHighestTemp() { return highestTemperature; }
float BMSModule::getLowestTemp()  { return lowestTemperature;  }

// ---------------------------------------------------------------------------
// Address / existence
// ---------------------------------------------------------------------------
void BMSModule::setAddress(int newAddr)
{
    if (newAddr < 0 || newAddr > MAX_MODULE_ADDR) return;
    moduleAddress = newAddr;
}

int  BMSModule::getAddress()      { return moduleAddress; }
bool BMSModule::isExisting()      { return exists;        }
void BMSModule::setExists(bool ex){ exists = ex;          }

// ---------------------------------------------------------------------------
// Configuration setters
// ---------------------------------------------------------------------------
void BMSModule::setModuleVoltage(float v) { moduleVolt = v; }
void BMSModule::setFaults(uint8_t f)     { faults = f; }
void BMSModule::setAlerts(uint8_t a)     { alerts = a; }

void BMSModule::settempsensor(int tempsensor) { sensor     = tempsensor; }
void BMSModule::setIgnoreCell(float Ignore)   { IgnoreCell = Ignore;     }
void BMSModule::setNumCells(int n)            { if (n >= 1 && n <= 16) numCells = (uint8_t)n; }
int  BMSModule::getNumCells() const           { return numCells; }

// Direct cell voltage write — used by CAN-sourced slave types (BMW i3 etc.)
void BMSModule::setCellVoltage(int cell, float v)
{
    if (cell < 0 || cell >= 16) return;
    cellVolt[cell] = v;
    if (v > IgnoreCell) {
        if (v < lowestCellVolt[cell])  lowestCellVolt[cell]  = v;
        if (v > highestCellVolt[cell]) highestCellVolt[cell] = v;
    }
}

void BMSModule::setTemperature(int idx, float t)
{
    if (idx < 0 || idx > 1) return;
    temperatures[idx] = t;
    if (t < lowestTemperature)  lowestTemperature  = t;
    if (t > highestTemperature) highestTemperature = t;
}
