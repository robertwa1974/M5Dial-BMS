#pragma once
// =============================================================================
// CANManager.h  v7 - SimpBMS-compatible CAN TX + CAN RX
//
// TX: SimpBMS frames 0x351/355/356/35A/35E/35F/35E/35F + extended 0x373
//     gated on externalDeviceSeen (any non-CSC frame received)
// RX: FreeRTOS task on core 0 decodes BMW i3/PHEV/VW frames, charger heartbeat
//
// PHEV TX: second FreeRTOS task (PHEV mode only) sends 0x080|slot every 50ms.
//   Without these poll commands the PHEV CSC modules transmit nothing.
//
// VW TX: third FreeRTOS task (VW CMU mode only) sends the two 0x0BA poll frames
//   every 500 ms, with the same empty-bus grace/backoff as BMWI3BUS, and the
//   balance commands handed over by loop() through a one-slot queue.
// =============================================================================
#include <Arduino.h>
#include <driver/twai.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "bms_config.h"   // for PHEV_MAX_MODS, BMW_PHEV_MAX_MODS
#include "CRC8.h"         // for PHEV poll command CRC
#include "VWProtocol.h"   // VW CMU frame decoders (pure functions)
#include "PacificaProtocol.h"  // Pacifica BPCM frame decoders (pure functions)

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
    uint32_t error;        // 32-bit error/status word from 0x0X0 status frame
    uint16_t balstat;      // active balance bits from status frame
    bool     fresh;
    uint32_t lastSeenMs;
};

// ---------------------------------------------------------------------------
// VW CMU (MEB / MQB) staging data
// Index 0 unused; 1..VW_MAX_MODS = module number from the cell-frame ID table.
// ---------------------------------------------------------------------------
struct VwSlaveData {
    float    cellV[VW_CELLS_PER_MOD];  // 13 cells max (modules 1-9), 12 on 10-16
    float    temp[VW_TEMPS_PER_MOD];   // VW_TEMP_INVALID until a sensor reports
    uint32_t balstat;                  // MEB balance status bits (0x16A9547x or type 1)
    uint8_t  subSeen;                  // sub-frames ever received (bit n = sub-frame n)
    bool     fresh;
    uint32_t lastSeenMs;               // last guard-passing CELL frame (presence)
};

// ---------------------------------------------------------------------------
// Chrysler Pacifica BPCM staging data
// Index 0 unused; 1..PAC_MAX_MODS = module number from the cell-frame ID.
// ---------------------------------------------------------------------------
struct PacSlaveData {
    float    cellV[PAC_CELLS_PER_MOD];
    float    temp[PAC_TEMPS_PER_MOD];  // PAC_TEMP_INVALID until reported
    uint8_t  subSeen;                  // sub-frames received since last presence update
    bool     fresh;
    uint32_t lastSeenMs;               // set when all 4 sub-frames (16 cells) have arrived
};

// Pack-level BPCM values (not per module)
struct PacPackData {
    float     coolantOutlet;           // PAC_TEMP_INVALID until 0x307 seen
    float     coolantInlet;
    PacStatus status;                  // 0x150 (valid once statusSeenMs != 0)
    uint32_t  statusSeenMs;
    uint32_t  lastFrameMs;             // any decoded BPCM frame
};

// ---------------------------------------------------------------------------
// VW CMU balance request: loop() -> VW CAN task, through a one-slot queue
// (xQueueOverwrite: the newest request replaces one not yet sent).
// ---------------------------------------------------------------------------
struct VwBalRequest {
    bool     stop;                          // true = stop all 12 modules (masks ignored)
    uint16_t modules;                       // bit m = send a command to module m (1..12)
    uint16_t mask[VW_MAX_BAL_MODS + 1];     // cells to bleed per module (bit i = cell i+1)
};

class CANManager {
public:
    CANManager();
    bool begin();
    void end();
    bool isRunning();

    // TX
    void sendBatterySummary();
    // extd = true sends a 29-bit extended ID (VW CMU temperature/balance frames)
    void sendFrame(uint32_t id, uint8_t *data, uint8_t len, bool extd = false);

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

    // VW CMU slave data (addr 1..VW_MAX_MODS)
    bool getVwSlaveData(int addr, VwSlaveData &out);
    // VW balance commands (EXTENDED IDs). Called from loop(); the frames are
    // sent by the VW CAN task so loop() never blocks on the 1 ms gaps.
    // modules: bit m = module m (1..12) gets mask[m]; absent modules get none.
    void requestVwBalance(uint16_t modules, const uint16_t *mask);
    void requestVwBalanceStop();

    // Pacifica BPCM data (addr 1..PAC_MAX_MODS) and pack-level values
    bool getPacSlaveData(int addr, PacSlaveData &out);
    PacPackData getPacPackData() const;

private:
    bool         running;
    TaskHandle_t rxTaskHandle;
    TaskHandle_t phevCmdTaskHandle;   // PHEV poll task (nullptr in non-PHEV modes)
    TaskHandle_t i3BusCmdTaskHandle;  // BMWI3BUS periodic command task (nullptr otherwise)
    TaskHandle_t vwPollTaskHandle;    // VW CMU 0x0BA poll task (nullptr otherwise)

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
    PhevSlaveData phevdata[PHEV_MAX_MODS + 1];
    struct PhevCellAcc {
        float   cells[16];
        uint8_t framesRx;   // bits 0..5 = sub-frames Id1..Id6; complete = 0x3F
    } phevAcc[PHEV_MAX_MODS + 1];

    // PHEV command sequencer state
    uint8_t  phevNextMod;    // 0..BMW_PHEV_MAX_MODS-1, cycles with each poll
    uint8_t  phevMesCycle;   // 0x0..0xF rolling counter (upper nibble of buf[6])
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

    // VW CMU staging. Cells are written straight into vwdata as each
    // sub-frame arrives; vwFramesRx only tracks presence (sub-frames 0-1).
    VwSlaveData vwdata[VW_MAX_MODS + 1];
    uint8_t     vwFramesRx[VW_MAX_MODS + 1];   // bits 0..3 = sub-frames since last presence update
    uint32_t vwLastReplySeenMs;       // millis() of the last decoded VW frame
    uint32_t vwTxStartMs;             // millis() when polling began (backoff timing)

    // VW RX - returns true if the frame was a VW frame (consumed)
    bool processVwFrame(const twai_message_t &msg);

    // Pacifica BPCM staging (RX only - nothing is transmitted to the BPCM)
    PacSlaveData pacdata[PAC_MAX_MODS + 1];
    PacPackData  pacPack;
    // Pacifica RX - returns true if the frame was a known BPCM frame (consumed)
    bool processPacFrame(const twai_message_t &msg);

    // VW TX - 0x0BA poll pair, with empty-bus grace/backoff
    void sendVwPoll();
    // VW TX - balance frames (VW CAN task only)
    QueueHandle_t vwBalQueue;         // one-slot VwBalRequest queue (VW mode only)
    void sendVwBalanceRequest(const VwBalRequest &req);
    void sendVwBalance(int mod, uint16_t mask);
    void sendVwBalanceStop();

    // PHEV TX
    void sendPhevCommand();
    void sendPhevResetIDs();

    // FreeRTOS tasks
    static void rxTaskFn(void *param);
    static void phevCmdTaskFn(void *param);
    static void i3BusCmdTaskFn(void *param);
    static void vwPollTaskFn(void *param);
};
