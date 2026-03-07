#pragma once

// =============================================================================
// TeslaBMS M5Dial - bms_config.h
// Target: M5Dial (M5StampS3, ESP32-S3FN8)
// v5: 20-module support, LVGL, WiFi, CAN, active balancing
// =============================================================================

// ---------------------------------------------------------------------------
// USER CONFIGURATION - update these for your pack
// ---------------------------------------------------------------------------
#define BMS_NUM_SERIES           2      // Tesla modules in series
#define BMS_NUM_PARALLEL         1      // Tesla modules in parallel
#define BMS_BALANCE_VOLTAGE_MIN  4.0f
#define BMS_BALANCE_VOLTAGE_DELTA 0.04f

// ---------------------------------------------------------------------------
// MODULE LIMIT
// Protocol supports 62 addresses (0x01-0x3E). Reduce to 20 for faster
// findBoards() scan: 20 * 25ms = 500ms vs 62 * 25ms = 1.55s
// ---------------------------------------------------------------------------
#define MAX_MODULE_ADDR         20

// ---------------------------------------------------------------------------
// POWER HOLD  *** MUST be set HIGH as the very first thing in setup() ***
// ---------------------------------------------------------------------------
#define PIN_POWER_HOLD          46

// ---------------------------------------------------------------------------
// CMU UART - Grove Port A (confirmed working, no iomux conflict)
// Grove Port B (GPIO1/GPIO2) avoided: iomux conflict with M5.begin()
// ---------------------------------------------------------------------------
#define BMS_SERIAL_RX_PIN       13    // GPIO13 Grove A Yellow - CMU TX
#define BMS_SERIAL_TX_PIN       15    // GPIO15 Grove A White  - CMU RX
#define BMS_BAUD_RATE           612500

// ---------------------------------------------------------------------------
// CAN (TWAI) - Grove Port B
// Connect SN65HVD230 or MCP2551 CAN transceiver:
//   Grove B Pin 3 (GPIO1) = CAN TX  (Yellow wire)
//   Grove B Pin 4 (GPIO2) = CAN RX  (White wire)
//   Grove B Pin 2 (5V)    = VCC
//   Grove B Pin 1 (GND)   = GND
// TWAI peripheral is distinct from UART; no iomux conflict on these pins.
// ---------------------------------------------------------------------------
#define PIN_CAN_TX              1     // GPIO1 Grove B Yellow
#define PIN_CAN_RX              2     // GPIO2 Grove B White
#define CAN_BAUD_RATE           500000

// ---------------------------------------------------------------------------
// ACTIVE BALANCING - Drive mode inhibit
// Connect to vehicle drive signal (12V or 3.3V via voltage divider).
// HIGH = vehicle in drive -> balancing disabled
// LOW  = vehicle parked   -> balancing enabled
// Use StampS3 expansion pad or external GPIO wire
// ---------------------------------------------------------------------------
#define PIN_DRIVE_INHIBIT       38    // INPUT_PULLDOWN - HIGH disables balancing
#define BALANCE_INHIBIT_ACTIVE  HIGH  // Logic level that disables balancing

// ---------------------------------------------------------------------------
// WiFi credentials - change before flashing or use WIFI=SSID:PASS serial cmd
// ---------------------------------------------------------------------------
#define WIFI_SSID_DEFAULT       "TeslaBMS"
#define WIFI_PASS_DEFAULT       "teslapack"
#define WIFI_AP_MODE            true   // true=access point, false=station mode

// ---------------------------------------------------------------------------
// M5Dial built-in peripherals (managed by M5Unified)
//   GC9A01 display   : GPIO4(DC) 5(MOSI) 6(SCK) 7(CS) 8(RST) 9(BL)
//   Rotary encoder   : GPIO40(A) 41(B) 42(BTN)
//   FT3267 touch     : I2C GPIO11(SDA) 12(SCL) 14(INT)
//   BM8563 RTC       : I2C GPIO11(SDA) 12(SCL) addr 0x51
//   Buzzer           : GPIO3
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Serial port macros
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include "HardwareSerial.h"

#define SERIALCONSOLE   Serial
extern HardwareSerial SERIALBMS;

// ---------------------------------------------------------------------------
// BQ76PL536 register map
// ---------------------------------------------------------------------------
#define REG_DEV_STATUS      1
#define REG_GPAI            1
#define REG_VCELL1          3
#define REG_VCELL2          5
#define REG_VCELL3          7
#define REG_VCELL4          9
#define REG_VCELL5          0xB
#define REG_VCELL6          0xD
#define REG_TEMPERATURE1    0xF
#define REG_TEMPERATURE2    0x11
#define REG_ALERT_STATUS    0x20
#define REG_FAULT_STATUS    0x21
#define REG_COV_FAULT       0x22
#define REG_CUV_FAULT       0x23
#define REG_ADC_CTRL        0x30
#define REG_IO_CTRL         0x31
#define REG_BAL_CTRL        0x32
#define REG_BAL_TIME        0x33
#define REG_ADC_CONV        0x34
#define REG_ADDR_CTRL       0x3B

// ---------------------------------------------------------------------------
// CMU Slave Type
// ---------------------------------------------------------------------------
enum CmuType : uint8_t {
    CMU_TESLA  = 0,   // BQ76PL536A UART daisy-chain (default)
    CMU_BMW_I3 = 1,   // BMW i3 CSC modules via CAN
};

// ---------------------------------------------------------------------------
// CAN-based balance inhibit
// When enabled, balancing is also paused when no charger heartbeat frame has
// been seen within CHARGER_TIMEOUT_MS milliseconds.
// ---------------------------------------------------------------------------
#define CHARGER_TIMEOUT_MS       5000    // ms silence = drive/idle mode
#define DEFAULT_CHARGER_HB_ID    0x305   // CAB300 current sensor / Victron heartbeat
#define DEFAULT_CAN_INHIBIT      0       // 0=GPIO only, 1=GPIO+CAN

// BMW i3 CSC CAN IDs
// Each CSC broadcasts three frames per cycle; frame set per module:
//   0x3D1+n  cells 1-4  (2 bytes each, 1mV/bit, big-endian)
//   0x3B1+n  temperatures (2 bytes each, 0.1°C/bit, signed)
// Module index n = 0..7 (module addresses 1..8 map to IDs 0x3D1..0x3D8)
#define BMW_I3_CELL_BASE    0x3D1
#define BMW_I3_TEMP_BASE    0x3B1
#define BMW_I3_WAKE_ID      0x130
#define BMW_I3_CELLS_PER_MOD  12    // 12 cells per i3 CSC module
#define BMW_I3_MAX_MODS       8     // max 8 CSC modules in a pack

// ---------------------------------------------------------------------------
// EEPROM / NVS settings
// Bump EEPROM_VERSION whenever EEPROMSettings layout changes
// ---------------------------------------------------------------------------
#define EEPROM_VERSION      0x17    // v6: cmuType, canInhibitEnabled, chargerHeartbeatID
#define EEPROM_PAGE         0

#define DEFAULT_OVER_V          4.20f
#define DEFAULT_UNDER_V         3.00f
#define DEFAULT_OVER_T          55.0f
#define DEFAULT_UNDER_T        -20.0f
#define DEFAULT_CHARGE_T        45.0f
#define DEFAULT_DIS_T          -20.0f
#define DEFAULT_BALANCE_V        4.10f
#define DEFAULT_BALANCE_HYST     0.02f
#define DEFAULT_IGNORE_VOLT      2.5f
#define DEFAULT_IGNORE_TEMP      0
#define DEFAULT_IGNORE_TEMP_THRESH  -70.0f
#define DEFAULT_NUM_CELLS        6       // cells per module (4,5,6)
#define DEFAULT_SOC_LO           18.0f   // V per module at 0% SoC
#define DEFAULT_SOC_HI           25.2f   // V per module at 100% SoC

typedef struct {
    uint8_t  version;
    uint8_t  checksum;
    uint32_t canSpeed;
    uint8_t  batteryID;
    uint8_t  logLevel;
    float    OverVSetpoint;
    float    UnderVSetpoint;
    float    OverTSetpoint;
    float    UnderTSetpoint;
    float    ChargeTSetpoint;
    float    DisTSetpoint;
    uint8_t  IgnoreTemp;
    float    IgnoreVolt;
    float    balanceVoltage;
    float    balanceHyst;
    float    IgnoreTempThresh; // ignore temps below this C, default -70
    uint8_t  wifiEnabled;          // 1 = WiFi on at boot
    uint8_t  balancingEnabled;     // 1 = auto-balance enabled
    char     wifiSSID[32];
    char     wifiPass[32];
    uint8_t  numCells;       // global default: active cells per module (4,5,6)
    uint8_t  numSeries;      // modules in series
    uint8_t  numParallel;    // modules in parallel
    float    socLo;          // V per module at 0% SoC
    float    socHi;          // V per module at 100% SoC
    // Per-module cell count override.  0 = use numCells global default.
    // Index 1..MAX_MODULE_ADDR maps directly to module address.
    uint8_t  moduleCells[MAX_MODULE_ADDR + 1];
    // v6: CMU type and CAN-based inhibit
    uint8_t  cmuType;              // CmuType enum: 0=Tesla UART, 1=BMW i3 CAN
    uint8_t  canInhibitEnabled;    // 1 = pause balancing when charger silent on CAN
    uint32_t chargerHeartbeatID;   // CAN ID to watch for charger/inverter presence
} EEPROMSettings;
