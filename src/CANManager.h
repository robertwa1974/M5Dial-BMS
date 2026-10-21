#pragma once
// =============================================================================
// CANManager.h  v7 - SimpBMS-compatible CAN TX + CAN RX
//
// TX: SimpBMS frames 0x351/355/356/35A/35E/35F/35E/35F + extended 0x373
//     gated on externalDeviceSeen (any non-CSC frame received)
// RX: FreeRTOS task on core 0 decodes BMW i3/PHEV frames, charger heartbeat
//
// PHEV TX: second FreeRTOS task (PHEV mode only) sends 0x080|slot every 50ms.
//   Without these poll commands the PHEV CSC modules transmit nothing.
// =============================================================================
#include <Arduino.h>
#include <driver/twai.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bms_config.h"   // for PHEV_MAX_MODS, BMW_PHEV_MAX_MODS
#include "CRC8.h"         // for PHEV poll command CRC
#include "PhevProtocol.h" // PHEV frame builders / decoders (pure functions)

// ---------------------------------------------------------------------------
// BMW i3 / Mini-E / Bus Pack staging data
// ---------------------------------------------------------------------------
#define I3_MAX_MODS 9   // index 0 unused; 1..8 = module address
struct I3SlaveData {
    float    cellV[12];
    float    temp[2];
    bool     fresh;
    uint32_t lastSeenMs;
};

// ---------------------------------------------------------------------------
// BMW PHEV SP06/SP41/SP44 staging data
// PHEV_MAX_MODS (= BMW_PHEV_MAX_MODS = 12) defined in bms_config.h
// ---------------------------------------------------------------------------
struct PhevSlaveData {
    float    cellV[16];    // up to 16 cells per module (SP44 uses 8)
    float    temp[4];      // 4 temperature sensors per module
    uint32_t error;        // 32-bit error/status word from 0x10X status frame
    uint16_t balstat;      // active balance bits from status frame
    bool     fresh;
    uint32_t lastSeenMs;
};

class CANManager {
public:
    CANManager();
    bool begin();
    void end();
    bool isRunning();

    // TX
    void sendBatterySummary();
    void sendFrame(uint32_t id, uint8_t *data, uint8_t len);

    // RX dispatch
    void processRxFrame(const twai_message_t &msg);

    // Charger / inverter presence
    bool     getChargerActive();
    float    getCanCurrentA();
    bool     hasExternalDevice() const;
    uint32_t getI3LastSeen(int addr) const;

    // BMW i3 / Mini-E / Bus Pack slave data
    bool getI3SlaveData(int addr, I3SlaveData &out);
    void sendI3WakeFrame();

    // TX - BMWI3BUS periodic command (CMU_BMW_I3_BUS, called from periodic task)
    // Sends confirmed SME format: C7 10 00 [D4] 20 00 [counter] [CRC] on 0x080-0x087
    void sendBMWI3BUSCommand();

    // BMW PHEV slave data
    bool getPhevSlaveData(int slot, PhevSlaveData &out);
    // BMW PHEV: reset all CSC IDs and assign 0..n to every CSC that answers
    // (console PHEVASSIGN=1). Runs in the PHEV task; progress is logged.
    void startPhevIdAssignment();
    bool isPhevAssigning() const { return phevAssignActive; }

private:
    bool         running;
    TaskHandle_t rxTaskHandle;
    TaskHandle_t phevCmdTaskHandle;   // PHEV poll task (nullptr in non-PHEV modes)
    TaskHandle_t i3BusCmdTaskHandle;  // BMWI3BUS periodic command task (nullptr otherwise)

    uint32_t     lastChargerSeen;
    float        canCurrentA;
    bool         externalDeviceSeen;

    // BMW i3 staging + accumulator
    I3SlaveData   i3data[I3_MAX_MODS];
    struct I3CellAcc {
        float   cells[12];
        uint8_t framesRx;   // bits 0/1/2 = sub-frames 0/1/2
    } i3acc[I3_MAX_MODS];

    // BMW PHEV staging + accumulator
    // Index 0 unused; slots 1..PHEV_MAX_MODS map to module address (nextMod+1)
    // Cells are written straight into phevdata as each sub-frame arrives;
    // phevFramesRx only tracks presence (sub-frames 0-2, see PhevProtocol.h).
    PhevSlaveData phevdata[PHEV_MAX_MODS + 1];
    uint8_t       phevFramesRx[PHEV_MAX_MODS + 1];   // bits 0..5 = sub-frames since last presence update

    // PHEV command sequencer state
    uint8_t  phevNextMod;    // 0..BMW_PHEV_MAX_MODS-1, cycles with each poll
    uint8_t  phevMesCycle;   // 0x0..0xE rolling counter (upper nibble of buf[6]; never 0xF)
    uint8_t  phevTestCycle;  // 0..4, ramps up to enable voltage+temp measurement
    bool     phevBalCells;   // true = include balance target voltage in command

    // PHEV CRC helper (also reused for BMWI3BUS command checksum - same
    // BMW_PHEV_FINAL_XOR table, confirmed identical to T-CAN485's miniE_finalxor)
    CRC8    phevCrc8;
    uint8_t getPhevChecksum(uint32_t id, uint8_t *buf, int modIdx);
    uint8_t getI3BusChecksum(uint32_t id, uint8_t *buf, int modIdx);

    // BMWI3BUS command counter (independent of PHEV's cycle state)
    uint16_t bmwI3Bus_counter;   // cycle count: 0-3=init, 4+=steady state
    uint32_t i3BusLastReplySeenMs;  // millis() of the last real 0x100-0x1FF reply
    uint32_t i3BusTxStartMs;        // millis() when steady TX began (for backoff timing)

    // PHEV empty-bus backoff (same scheme as BMWI3BUS)
    uint32_t phevLastReplySeenMs;   // millis() of the last PHEV CSC frame
    uint32_t phevTxStartMs;         // millis() when polling began

    // PHEV ID assignment (BMWPhevBMS.ino:3031-3058, 3665-3760)
    volatile bool phevAssignRequested;  // set by console, consumed by the PHEV task
    bool     phevAssignActive;
    volatile uint8_t phevUnassigned;    // 0xF0 replies since the last find-unassigned
    uint8_t  phevDmc[8];                // DMC of the last unassigned CSC that answered
    uint8_t  phevNextId;                // next ID to hand out
    uint32_t phevAssignStepMs;          // last 500 ms assignment step
    uint32_t phevAssignActivityMs;      // last reset / assignment
    uint32_t phevUnassignedLogMs;       // rate limit for the "run PHEVASSIGN" hint
    void phevAssignStep();
    void sendPhevFindUnassigned();
    void sendPhevAssignId(const uint8_t *dmc, uint8_t id);

    // PHEV TX
    void sendPhevCommand();
    void sendPhevResetIDs();

    // FreeRTOS tasks
    static void rxTaskFn(void *param);
    static void phevCmdTaskFn(void *param);
    static void i3BusCmdTaskFn(void *param);
};
