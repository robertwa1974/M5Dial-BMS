#pragma once
// =============================================================================
// CANManager.h  v7 - SimpBMS-compatible CAN TX + CAN RX
//
// TX: SimpBMS frames 0x351/355/356/35A/35E/35F + extended 0x372/373/374
//     gated on externalDeviceSeen (any non-CSC frame received)
// RX: FreeRTOS task on core 0 decodes BMW i3/PHEV frames, charger heartbeat
// =============================================================================
#include <Arduino.h>
#include <driver/twai.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
// BMW PHEV SP06/SP41 staging data
// NOTE: PHEV_MAX_MODS and PHEV_CELLS are defined in bms_config.h — not here
// ---------------------------------------------------------------------------
struct PhevSlaveData {
    float    cellV[16];   // 16 cells max
    float    temp[4];     // 4 temperature sensors
    uint16_t errorWord;
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

    // BMW PHEV slave data
    bool getPhevSlaveData(int slot, PhevSlaveData &out);

private:
    bool         running;
    TaskHandle_t rxTaskHandle;

    uint32_t     lastChargerSeen;
    float        canCurrentA;
    bool         externalDeviceSeen;

    I3SlaveData   i3data[I3_MAX_MODS];
    PhevSlaveData phevdata[7];   // slots 1..6; index 0 unused

    struct I3CellAcc {
        float   cells[12];
        uint8_t framesRx;
    } i3acc[I3_MAX_MODS];

    static void rxTaskFn(void *param);
};
