#pragma once
// =============================================================================
// CANManager.h  v6 - SimpBMS-compatible CAN TX + CAN RX
//
// TX (same as v5): SimpBMS frames 0x351/355/356/35A/35E/35F every 1s
// RX (new v6):
//   - FreeRTOS task calls twai_receive() continuously on core 0
//   - Charger/inverter heartbeat detection -> balance inhibit via CAN
//   - BMW i3 CSC cell/temp frames decoded into I3SlaveData staging buffer
//
// Hardware: SN65HVD230 or MCP2551 on Grove Port B
//   GPIO1 = CAN TX, GPIO2 = CAN RX
// =============================================================================
#include <Arduino.h>
#include <driver/twai.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// BMW i3 staging data populated by CAN RX task, read by BMSModuleManager
#define I3_MAX_MODS 9   // index 0 unused; 1..8 = module address
struct I3SlaveData {
    float    cellV[12];     // cell voltages in volts
    float    temp[2];       // temperatures in degC
    bool     fresh;         // true = new data since last read
    uint32_t lastSeenMs;    // millis() of last received frame
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

    // RX dispatch called by RX task
    void processRxFrame(const twai_message_t &msg);

    // Charger/inverter presence
    bool  getChargerActive();
    float getCanCurrentA();

    // BMW i3 slave data access
    bool  getI3SlaveData(int addr, I3SlaveData &out);
    void  sendI3WakeFrame();

private:
    bool         running;
    TaskHandle_t rxTaskHandle;

    uint32_t     lastChargerSeen;
    float        canCurrentA;

    I3SlaveData  i3data[I3_MAX_MODS];

    // Accumulator for multi-frame i3 cell sets
    struct I3CellAcc {
        float   cells[12];
        uint8_t framesRx;   // bitmask bits 0/1/2 = sub-frame 0/1/2 received
    } i3acc[I3_MAX_MODS];

    static void rxTaskFn(void *param);
};
