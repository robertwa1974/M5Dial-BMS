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
#define PIN_CAN_TX              2     // GPIO1 Grove B Yellow
#define PIN_CAN_RX              1     // GPIO2 Grove B White
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
#define WIFI_SSID_DEFAULT       "M5DialBMS"
#define WIFI_PASS_DEFAULT       "M5DialBMS"
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
    CMU_TESLA        = 0,   // Tesla BQ76PL536A UART daisy-chain
    CMU_BMW_I3       = 1,   // BMW i3 CSC standard (0x3D1-0x3D8 cells, 0x3B1-0x3B8 temps)
    CMU_BMW_I3_BUS   = 2,   // BMW i3 bus-pack variant (0x080 TX keepalive, 0x100-0x17F RX)
    CMU_BMW_MINIE    = 3,   // BMW Mini-E CSC (0x080 TX, 0x0A0-0x17F RX)
    CMU_BMW_PHEV     = 4,   // BMW PHEV SP06/SP41/SP44 — polled 50ms, 16 cells, 4 temps
    CMU_VW           = 5,   // VW CMU (MEB/MQB) — 0x0BA poll, 0x1B0-0x1EE cells, ext-ID temps/balance
};
#define CMU_TYPE_MAX            CMU_VW  // highest valid CmuType (range checks in UI/console)

// ---------------------------------------------------------------------------
// CAN-based balance inhibit
// When enabled, balancing is also paused when no charger heartbeat frame has
// been seen within CHARGER_TIMEOUT_MS milliseconds.
// ---------------------------------------------------------------------------
#define CHARGER_TIMEOUT_MS       5000    // ms silence = drive/idle mode
#define DEFAULT_CHARGER_HB_ID    0x305   // CAB300 current sensor / Victron heartbeat
#define DEFAULT_CAN_INHIBIT      0       // 0=GPIO only, 1=GPIO+CAN

// BMW i3 standard CSC CAN IDs
// Each CSC broadcasts three sub-frames per cycle; one frame set per module:
//   0x3D1+n  cells 1-12 (3 sub-frames, 4 cells each, 1mV/bit, big-endian)
//   0x3B1+n  temperatures (2 bytes each, 0.1°C/bit, signed)
// Module index n = 0..7 (module addresses 1..8 map to IDs 0x3D1..0x3D8)
#define BMW_I3_CELL_BASE      0x3D1
#define BMW_I3_TEMP_BASE      0x3B1
#define BMW_I3_WAKE_ID        0x130
#define BMW_I3_CMD_ID         0x0A0  // management bus TX — find/assign/reset CSC IDs
#define BMW_I3_BAL_RESET_ID   0x0B0  // balance reset TX frame
#define BMW_I3_CELLS_PER_MOD  12     // 12 cells per i3 CSC module
#define BMW_I3_MAX_MODS       8      // hardware max: 8 CSC modules per CAN bus
                                      // Standard i3: IDs 0x3D1-0x3D8 (8 fixed)
                                      // i3 bus:      TX loop hardcoded slot < 8
                                      // Mini-E:      lower nibble 0-7 in practice

// ---------------------------------------------------------------------------
// Mini-E and BMWI3BUS shared TX base
// Both variants receive commands on 0x080+slot (one frame per module slot)
// ---------------------------------------------------------------------------
#define BMW_CSC_CMD_BASE        0x080   // TX: BMS->CSC command/keepalive

// ---------------------------------------------------------------------------
// BMWI3BUS variant RX frame IDs
// Frame ID structure: bits[7:4] = type, bits[3:0] = module address (0-based)
//   0x10N = heartbeat/status
//   0x11N = init ack
//   0x12N = cells 1-3   (LE 16-bit, 1 mV/bit)
//   0x13N = cells 4-6
//   0x14N = cells 7-9
//   0x15N = cells 10-12
//   0x16N = raw NTC ADC (3x LE 16-bit thermistors)
//   0x17N = status2: data[4] = temp + 40 (degC)  [NOTE: 0-indexed byte 4]
//   0x1CN = balance/fault flags
//   0x1DN = additional status flags
// ---------------------------------------------------------------------------
#define BMWI3BUS_CELL_BASE      0x100   // lowest cell frame ID for bus variant
#define BMWI3BUS_FRAME_MAX      0x1FF   // covers all bus variant frames

// ---------------------------------------------------------------------------
// Mini-E variant RX frame IDs
// Frame ID structure: upper nibble = type, lower nibble = module (0-based addr)
//   type 0x2 = cells 1-3   voltage: lo + (hi & 0x3F) * 256, result in mV
//   type 0x3 = cells 4-6
//   type 0x4 = cells 7-9
//   type 0x5 = cells 10-12
// Temperature: 0x170+mod_addr, data[0]-40=T1, data[1]-40=T2 (degC)
// ---------------------------------------------------------------------------
#define MINIE_CELL_BASE         0x0A0   // Mini-E: lowest cell frame ID
#define MINIE_CELL_MAX          0x15F   // Mini-E: highest cell frame ID
#define MINIE_TEMP_BASE         0x170   // Mini-E temperature frames
#define MINIE_TEMP_MAX          0x17F
#define MINIE_CELLS_PER_MOD     12

// ---------------------------------------------------------------------------
// BMW PHEV SP06/SP41/SP44 CSC CAN IDs
// Polled master/slave: BMS sends 0x080|slot every BMW_PHEV_CMD_RATE_MS (50ms).
// One module per call; with 12 slots polled each module is hit every 600ms.
// Variants (all use the same on-wire protocol, only module count differs):
//   SP06 (F30/F15/G30/G12 pre-LCI):  6 modules of 16 cells  → 300ms cycle
//   SP41 (G12 LCI 745e):              6 modules of 16 cells  → 300ms cycle
//   SP44 (X5 G05 45e):               12 modules of  8 cells  → 600ms cycle
// Polling all 12 slots is safe on 6-module packs — modules at unassigned
// addresses silently ignore the poll. Confirmed on hardware by mane2 on the
// openinverter forum (BMW BMS modules thread, June 2025).
//   RX 0x0A0-0x0AF  status/error frames per module (type nibble 0xA)
//     bytes 0-3 LE = 32-bit error word, bytes 4-5 LE = balstat
//   RX 0x120-0x17F  cell voltages (6 sub-frame groups per module, LE 14-bit)
//     type nibble 2-7 of lower byte: sub 0-5, 3 cells each (16 active of 18)
//     Voltage encoding: lo + (hi & 0x3F) * 256 = millivolts
//     Commit condition: framesRx == 0x3F (all 6 sub-frames received)
//     Cell update suppressed when status frame indicates balancing active
//   RX 0x180-0x18F  temperatures (4 sensors per module)
//     bytes 0-3: each uint8, value - 40 = degC
// ---------------------------------------------------------------------------
#define BMW_PHEV_CELL_BASE      0x120   // PHEV: lowest cell sub-frame ID
#define BMW_PHEV_CELL_MAX       0x17F   // PHEV: highest cell sub-frame ID
#define BMW_PHEV_STATUS_BASE    0x0A0   // PHEV: status/error frames (type 0xA)
#define BMW_PHEV_TEMP_BASE      0x180   // PHEV: temperature frames
#define BMW_PHEV_TEMP_MAX       0x18F   // PHEV: highest temperature frame ID
#define BMW_PHEV_CMD_BASE       0x080   // PHEV: TX command base (shared with Mini-E/i3bus)
#define BMW_PHEV_MGMT_ID        0x0A0   // PHEV: management reset TX ID
#define BMW_PHEV_CELLS_PER_MOD  16      // up to 16 cells per CSC module (SP44 uses 8 of 16)
#define BMW_PHEV_TEMPS_PER_MOD  4       // 4 temperature sensors per module
#define BMW_PHEV_MAX_MODS       12      // hardware max: 12 CSC modules per bus (SP44)
                                        // SP06/SP41 packs use 6; extra polls are harmless
#define BMW_PHEV_CMD_RATE_MS    50      // PHEV: command period (ms) — one module per call
#define BMW_PHEV_TIMEOUT_MS     3000    // module considered offline after this many ms
#define PHEV_MAX_MODS           BMW_PHEV_MAX_MODS   // alias used by CANManager arrays

// CRC finalxor table — one entry per module index 0..11 (from Tom-evnut/BMWPhevBMS).
// Used in CANManager::getPhevChecksum() to finalise the CRC8 of each poll frame.
// All 12 entries needed for SP44 (12 modules); SP06/SP41 only use indices 0..5.
// Defined in CANManager.cpp; declared extern here to avoid multiple-definition errors.
//
// Also reused (same table, same function) for BMWI3BUS and Mini-E command
// checksums: confirmed byte-identical to the T-CAN485 reference's local
// miniE_finalxor[12] table, so one shared source of truth here rather than
// duplicating it. Revert to a separate table if the variants ever diverge.
extern const uint8_t BMW_PHEV_FINAL_XOR[12];

// ---------------------------------------------------------------------------
// BMWI3BUS / Mini-E command interval (shared keepalive burst period)
// ---------------------------------------------------------------------------
#define BMW_CSC_CMD_INTERVAL_MS  24    // keepalive/command burst period (ms)

// ---------------------------------------------------------------------------
// VW CMU (MEB / MQB e-Golf, GTE) CAN protocol
// Ported from Tom de Bree's VWBMSV2:
//   https://github.com/Tom-evnut/VW-bms  commit 4a6a5b9 (2024-03-29)
// Citations below are "file:line" inside VWBMSV2/ at that commit.
// Frame decoders live in VWProtocol.h (pure functions, host-testable).
//
// Bus: 500 kbit/s (VWBMSV2.ino:314 Can0.begin(500000)) - same as TWAI config.
//
// Poll (VWBMSV2.ino:190 controlid, :3104-3129 sendcommand(), called once per
// 500 ms main-loop block :758-828):
//   two STANDARD frames on 0x0BA, 1 ms apart. Broadcast, not per module:
//     00 00 00 00 00 00 00 00
//     45 01 28 00 00 00 00 30
//   VWBMSV2.ino:191 declares moduleidstart = 0x1CC but never uses it. 0x1CC
//   is module 8 sub-frame 0, NOT a base address.
//
// Cell voltages - STANDARD IDs 0x1B0-0x1EE (BMSModuleManager.cpp:430-718).
//   The CAN ID encodes BOTH module and sub-frame; never use arrival order:
//     modules 1-9  : 0x1B0 + 4*(m-1)  + sub, sub 0..3  -> up to 13 cells
//     modules 10-16: 0x1D4 + 4*(m-10) + sub, sub 0..2  -> up to 12 cells
//                    (the reference's "one extender increment" block)
//   Encoding (BMSModule.cpp:99-240): 4 cells per sub-frame, 12-bit
//   nibble-packed, mV = raw + 1000. Sub-frame 3 carries only cell 13.
//   Guards (BMSModuleManager.cpp:687-702): sub 0-1 dropped if buf[2], buf[5]
//   or buf[7] == 0xFF; other subs dropped if buf[2] == 0xFF (module
//   initialising, or a "spoof module").
//   The reference skips cell decode while balancing (BMSModuleManager.cpp:432).
//   Here cells are always decoded (display, CAN TX); balance selection and
//   overvoltage checks use un-bled readings instead - see VWProtocol.h.
//
// Temperatures - EXTENDED IDs (VWBMSV2.ino:2864-2880,
// BMSModuleManager.cpp:388-428, BMSModule.cpp:51-97):
//   type 1:       0x1A555401-0x1A55543F
//   type 2 (MEB): 0x1A5555F0-0x1A5555FE
// Balance status (MEB) - EXTENDED 0x16A95471-0x16A9547F (VWBMSV2.ino:2855-2862,
//   BMSModuleManager.cpp:353-366, BMSModule.cpp:46-49)
// Balance command - EXTENDED, two frames per module 1-12
//   (BMSModuleManager.cpp:50-240, IDs in VW_BAL_CMD_ID_A/B in CANManager.cpp)
//
// Which of these blocks MEB modules actually use (and whether they need the
// 0x0BA poll at all) is still to be confirmed on the bench - see README.
// ---------------------------------------------------------------------------
#define VW_POLL_ID              0x0BA   // TX: broadcast poll (standard ID)
#define VW_CELL_ID_MIN          0x1B0   // RX: first cell frame (module 1, sub 0)
#define VW_CELL_ID_MAX          0x1EE   // RX: last cell frame (module 16, sub 2)
#define VW_TEMP1_ID_MIN         0x1A555401UL  // RX ext: type 1 temperatures
#define VW_TEMP1_ID_MAX         0x1A55543FUL
#define VW_TEMP2_ID_MIN         0x1A5555F0UL  // RX ext: type 2 (MEB) temperatures
#define VW_TEMP2_ID_MAX         0x1A5555FEUL
#define VW_BALSTAT_ID_MIN       0x16A95471UL  // RX ext: MEB balance status
#define VW_BALSTAT_ID_MAX       0x16A9547FUL
#define VW_MAX_MODS             16      // modules addressable by the cell-ID table
#define VW_MAX_BAL_MODS         12      // modules with a balance-command ID pair
#define VW_STD_MODS             9       // modules 1-9 have sub-frame 3 (cell 13)
#define VW_CELLS_PER_MOD        13      // max cells per module (modules 1-9)
#define VW_TEMPS_PER_MOD        3       // type 1 carries T0..T2
#define VW_POLL_INTERVAL_MS     500     // reference main-loop period (VWBMSV2.ino:758)
#define VW_BAL_TICK_MS          500     // balance state-machine tick (same loop)
#define VW_TIMEOUT_MS           3000    // module offline after this long without cell frames
#define VW_TEMP_INVALID         -100.0f // sensor absent (below IgnoreTempThresh default -70)
// Balancing vs. protection (deviation from the reference - see VWProtocol.h)
#define VW_SETTLE_MS            1000    // after a stop, readings count as un-bled after this
#define VW_SNAP_MAX_AGE_MS      3000    // un-bled snapshot trusted for OV checks for this long
#define VW_BLEED_OV_MARGIN      0.05f   // V: bleeding cells read low by up to this (bench-check)

// ---------------------------------------------------------------------------
// EEPROM / NVS settings
// Bump EEPROM_VERSION whenever EEPROMSettings layout changes
// ---------------------------------------------------------------------------
#define EEPROM_VERSION      0x19    // v7: 5-way cmuType (Tesla/i3/i3bus/MiniE/PHEV)
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
