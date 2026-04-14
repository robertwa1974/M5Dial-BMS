#pragma once
// =============================================================================
// CANManager.h  v7 - SimpBMS-compatible CAN TX + CAN RX
//
// TX (same as v6): SimpBMS frames 0x351/355/356/35A/35E/35F every 1s
// RX (new v7):
//   - FreeRTOS task calls twai_receive() continuously on core 0
//   - Charger/inverter heartbeat detection -> balance inhibit via CAN
//   - BMW i3 standard CSC (0x3D1-0x3D8 cells, 0x3B1-0x3B8 temps)
//   - BMW i3 bus-pack CSC (0x120-0x15N cells, 0x17N temp)
//   - BMW Mini-E CSC (0x120-0x15N cells via different voltage formula, 0x17N temp)
//
// TX (new v7):
//   - Mini-E periodic command   : sendMiniECommand()   every BMW_CSC_CMD_INTERVAL_MS
//   - BMWI3BUS periodic keepalive: sendBMWI3BUSCommand() every BMW_CSC_CMD_INTERVAL_MS
//   - i3 standard enumeration helpers: sendI3FindUnassigned/AssignID/ResetAllIDs/BalanceReset
//
// Hardware: SN65HVD230 or MCP2551 on Grove Port B
//   GPIO1 = CAN TX, GPIO2 = CAN RX
// =============================================================================
#include <Arduino.h>
#include <driver/twai.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------------------------------------------------------------------
// I3SlaveData — populated by CAN RX task, consumed by BMSModuleManager
// Shared by all three BMW CAN variants (standard i3, i3 bus, Mini-E).
// ---------------------------------------------------------------------------
// I3_MAX_MODS: index 0 is unused; valid slots are 1..I3_MAX_MODS-1
// Set to 17 to accommodate up to 16 modules (Mini-E/bus-pack can use 0-15 addr)
#define I3_MAX_MODS 17

struct I3SlaveData {
    float    cellV[12];      // cell voltages in volts
    float    temp[2];        // temperatures in degC
    bool     fresh;          // true = new data since last read
    uint32_t lastSeenMs;     // millis() of last received frame
    uint8_t  dmcBytes[8];    // raw bytes from 0x4A0 enumeration reply (i3 std)
};

class CANManager {
public:
    CANManager();
    bool begin();
    void end();
    bool isRunning();

    // TX — SimpBMS outbound summary frames
    void sendBatterySummary();
    void sendFrame(uint32_t id, uint8_t *data, uint8_t len);

    // RX dispatch called by RX task
    void processRxFrame(const twai_message_t &msg);

    // Charger/inverter presence
    bool  getChargerActive();
    float getCanCurrentA();

    // BMW i3 slave data access
    bool  getI3SlaveData(int addr, I3SlaveData &out);
    void  sendI3WakeFrame();

    // TX — Mini-E periodic command (call from main loop every BMW_CSC_CMD_INTERVAL_MS)
    void sendMiniECommand();

    // TX — BMWI3BUS periodic keepalive (call from main loop every BMW_CSC_CMD_INTERVAL_MS)
    // Sends 8 frames (0x080-0x087): init sequence for first 4 calls then steady state.
    void sendBMWI3BUSCommand();

    // TX — BMW i3 standard enumeration helpers
    void sendI3FindUnassigned();
    void sendI3AssignID(uint8_t newID, const uint8_t dmcBytes[8]);
    void sendI3ResetAllIDs(uint8_t maxID = 14);
    void sendI3BalanceReset();

    // Unassigned CSC detection (set by RX when 0x4A0 seen)
    bool           hasUnassignedCSC() const;
    void           clearUnassignedFlag();
    const uint8_t* getUnassignedDMC() const;

private:
    bool         running;
    TaskHandle_t rxTaskHandle;

    uint32_t     lastChargerSeen;
    float        canCurrentA;

    I3SlaveData  i3data[I3_MAX_MODS];

    // Accumulator for multi-frame cell sets (used by all CAN variants)
    struct I3CellAcc {
        float   cells[12];
        uint8_t framesRx;   // bitmask: bits 0-3 = sub-frame groups received
    } i3acc[I3_MAX_MODS];

    static void rxTaskFn(void *param);

    // Mini-E command sequencer state
    uint8_t  miniE_nextmes;    // 0x00..0x0B, wraps at 0x0C
    uint8_t  miniE_mescycle;   // 0x00..0x0F, wraps at 0x10
    uint8_t  miniE_testcycle;  // 0..4, ramps to enable measurements

    // Mini-E / BMWI3BUS CRC8 helper
    uint8_t  cscChecksum(uint32_t msgId, const uint8_t *buf,
                         uint8_t len, uint8_t slotIdx);

    // BMWI3BUS command cycle counter
    // 0-3 = init sequence (hardcoded D4/counter values)
    // 4+  = steady state (D4=0x50, counter increments 0x10 per burst)
    uint16_t bmwI3Bus_counter;

    // Standard i3 enumeration state
    bool    unassignedSeen;
    uint8_t unassignedDMC[8];
};
