// =============================================================================
// CANManager.cpp  v6
//
// CAN RX architecture:
//   A FreeRTOS task (core 0, 4KB stack) calls twai_receive() with 100ms
//   timeout. Received frames are dispatched to processRxFrame() which:
//     1. Updates lastChargerSeen / canCurrentA for balance inhibit
//     2. Decodes BMW i3 CSC frames (0x3B1-0x3B8, 0x3D1-0x3D8) into i3data[]
//
// BMW i3 CAN protocol (reverse-engineered, matched to Tom-evnut/BMWI3BMS):
//   0x3D1..0x3D8  Cell voltages, one ID per CSC module (n = ID - 0x3D0)
//     Each frame carries 4 cells as 16-bit big-endian, 1 mV/bit, in 8 bytes.
//     The i3 CSC broadcasts three sequential frames per scan cycle; we use
//     the DLC byte (data[7] or sub-frame counter in byte 0) to tell them apart.
//     Simplification: we accumulate all 3 frames (4 cells each = 12 cells)
//     into i3acc[], then commit when all 3 are seen.
//
//   0x3B1..0x3B8  Temperatures, one ID per CSC module
//     bytes 0-1 = temp1 (signed 16-bit, 0.1degC/bit)
//     bytes 2-3 = temp2 (signed 16-bit, 0.1degC/bit)
// =============================================================================
#include "CANManager.h"
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "Logger.h"
#include "WiFiManager.h"

extern BMSModuleManager bms;
extern EEPROMSettings   settings;

CANManager::CANManager()
    : running(false), rxTaskHandle(nullptr),
      lastChargerSeen(0), canCurrentA(0.0f)
{
    memset(i3data, 0, sizeof(i3data));
    memset(i3acc,  0, sizeof(i3acc));
}

// ---------------------------------------------------------------------------
// begin() - install TWAI driver and start RX task
// ---------------------------------------------------------------------------
bool CANManager::begin()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX,
        (gpio_num_t)PIN_CAN_RX,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Logger::error("CAN: twai_driver_install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Logger::error("CAN: twai_start failed");
        twai_driver_uninstall();
        return false;
    }
    running = true;

    // Start RX task pinned to core 0 (BMS poll runs on core 1)
    xTaskCreatePinnedToCore(
        rxTaskFn,
        "CAN_RX",
        4096,
        this,
        5,              // priority (higher than loop)
        &rxTaskHandle,
        0               // core 0
    );

    Logger::info("CAN started: TX=GPIO%d RX=GPIO%d 500kbps, RX task running",
                 PIN_CAN_TX, PIN_CAN_RX);

    // If BMW i3 mode, send wake frame
    if (settings.cmuType == CMU_BMW_I3) {
        sendI3WakeFrame();
    }

    return true;
}

// ---------------------------------------------------------------------------
// end() - stop RX task and uninstall TWAI driver
// ---------------------------------------------------------------------------
void CANManager::end()
{
    if (!running) return;
    running = false;
    if (rxTaskHandle) {
        vTaskDelete(rxTaskHandle);
        rxTaskHandle = nullptr;
    }
    twai_stop();
    twai_driver_uninstall();
    Logger::info("CAN stopped");
}

bool CANManager::isRunning() { return running; }

// ---------------------------------------------------------------------------
// rxTaskFn - FreeRTOS task: receive frames and dispatch
// ---------------------------------------------------------------------------
void CANManager::rxTaskFn(void *param)
{
    CANManager *self = (CANManager *)param;
    twai_message_t msg;
    while (self->running) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            self->processRxFrame(msg);
        }
    }
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// processRxFrame - dispatch incoming CAN frame
// ---------------------------------------------------------------------------
void CANManager::processRxFrame(const twai_message_t &msg)
{
    uint32_t id  = msg.identifier;
    uint8_t  dlc = msg.data_length_code;

    // --- Charger / inverter heartbeat detection ---
    // Any frame matching the configured ID is treated as charger presence.
    // Also accepts 0x305 (CAB300 current sensor) and 0x306 (CAB300 alt).
    if (id == settings.chargerHeartbeatID ||
        id == 0x305 || id == 0x306)
    {
        lastChargerSeen = millis();
        // CAB300: bytes 0-1 = current in 0.1A signed, byte 0 = MSB
        if ((id == 0x305 || id == 0x306) && dlc >= 2) {
            int16_t raw = (int16_t)((msg.data[0] << 8) | msg.data[1]);
            canCurrentA = raw * 0.1f;
        }
    }

    // --- BMW i3 CSC cell voltage frames ---
    // 0x3D1..0x3D8: one ID per module, 3 sub-frames per scan
    // Sub-frame identification: we look at byte 7 (sub-frame counter 0/1/2)
    if (id >= BMW_I3_CELL_BASE && id <= (BMW_I3_CELL_BASE + BMW_I3_MAX_MODS - 1)) {
        int mod = (int)(id - BMW_I3_CELL_BASE) + 1;  // 1..8
        if (mod < 1 || mod >= I3_MAX_MODS) return;

        // Sub-frame index in byte 7 (0, 1, 2 -> cells 0-3, 4-7, 8-11)
        uint8_t sf = (dlc >= 8) ? (msg.data[7] & 0x03) : 0;
        int base = sf * 4;

        for (int c = 0; c < 4 && (base + c) < 12; c++) {
            uint16_t raw = ((uint16_t)msg.data[c * 2] << 8) | msg.data[c * 2 + 1];
            i3acc[mod].cells[base + c] = raw * 0.001f;  // mV -> V
        }
        i3acc[mod].framesRx |= (1 << sf);

        // When all 3 sub-frames received, commit to i3data
        if (i3acc[mod].framesRx == 0x07) {
            memcpy(i3data[mod].cellV, i3acc[mod].cells, sizeof(i3acc[mod].cells));
            i3data[mod].fresh      = true;
            i3data[mod].lastSeenMs = millis();
            i3acc[mod].framesRx    = 0;
        }
        return;
    }

    // --- BMW i3 CSC temperature frames ---
    // 0x3B1..0x3B8: bytes 0-1 = temp1, bytes 2-3 = temp2 (signed 16-bit, 0.1degC)
    if (id >= BMW_I3_TEMP_BASE && id <= (BMW_I3_TEMP_BASE + BMW_I3_MAX_MODS - 1)) {
        int mod = (int)(id - BMW_I3_TEMP_BASE) + 1;
        if (mod < 1 || mod >= I3_MAX_MODS || dlc < 4) return;
        int16_t t1 = (int16_t)((msg.data[0] << 8) | msg.data[1]);
        int16_t t2 = (int16_t)((msg.data[2] << 8) | msg.data[3]);
        i3data[mod].temp[0] = t1 * 0.1f;
        i3data[mod].temp[1] = t2 * 0.1f;
        return;
    }

    // Feed web CAN log for any other frames
    wifiLogCAN(id, (uint8_t *)msg.data, dlc);
}

// ---------------------------------------------------------------------------
// Charger / current accessors
// ---------------------------------------------------------------------------
bool CANManager::getChargerActive()
{
    if (lastChargerSeen == 0) return false;
    return (millis() - lastChargerSeen) < CHARGER_TIMEOUT_MS;
}

float CANManager::getCanCurrentA() { return canCurrentA; }

// ---------------------------------------------------------------------------
// BMW i3 slave data access (safe copy for BMSModuleManager)
// ---------------------------------------------------------------------------
bool CANManager::getI3SlaveData(int addr, I3SlaveData &out)
{
    if (addr < 1 || addr >= I3_MAX_MODS) return false;
    out = i3data[addr];   // struct copy
    i3data[addr].fresh = false;
    return out.lastSeenMs > 0;
}

// ---------------------------------------------------------------------------
// sendI3WakeFrame - broadcast 0x130 to wake BMW i3 CSC modules
// ---------------------------------------------------------------------------
void CANManager::sendI3WakeFrame()
{
    uint8_t data[8] = {0};
    sendFrame(BMW_I3_WAKE_ID, data, 8);
    Logger::info("CAN: BMW i3 wake frame sent (0x%03X)", BMW_I3_WAKE_ID);
}

// ---------------------------------------------------------------------------
// sendFrame - transmit one CAN frame, log to web feed
// ---------------------------------------------------------------------------
void CANManager::sendFrame(uint32_t id, uint8_t *data, uint8_t len)
{
    if (!running) return;
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = id;
    msg.extd             = 0;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);
    esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(5));
    if (res != ESP_OK && Logger::isDebug())
        Logger::debug("CAN TX err 0x%02X on ID 0x%03X", res, id);
    wifiLogCAN(id, data, len);
}

// ---------------------------------------------------------------------------
// sendBatterySummary - all SimpBMS outbound frames
// ---------------------------------------------------------------------------
void CANManager::sendBatterySummary()
{
    if (!running) return;

    float packV   = bms.getPackVoltage();
    float lowC    = bms.getLowCellVolt();
    float highC   = bms.getHighCellVolt();
    float avgT    = bms.getAvgTemperature();
    bool  faulted = bms.isFaultedState();

    float socRange = settings.socHi - settings.socLo;
    float socPer   = (socRange > 0.01f)
        ? ((packV / (float)(settings.numSeries > 0 ? settings.numSeries : BMS_NUM_SERIES)) - settings.socLo)
          / socRange * 100.0f
        : 0.0f;
    if (socPer > 100.0f) socPer = 100.0f;
    if (socPer < 0.0f)   socPer = 0.0f;

    uint8_t data[8];
    int ns = settings.numSeries > 0 ? settings.numSeries : BMS_NUM_SERIES;

    // 0x351 - Voltage/current limits
    int16_t cvh = (int16_t)(settings.OverVSetpoint  * ns * 10.0f);
    int16_t cvl = (int16_t)(settings.UnderVSetpoint * ns * 10.0f);
    int16_t ccl = 500;
    int16_t dcl = 1000;
    data[0]=cvh&0xFF; data[1]=(cvh>>8)&0xFF;
    data[2]=ccl&0xFF; data[3]=(ccl>>8)&0xFF;
    data[4]=dcl&0xFF; data[5]=(dcl>>8)&0xFF;
    data[6]=cvl&0xFF; data[7]=(cvl>>8)&0xFF;
    sendFrame(0x351, data, 8);

    // 0x355 - SOC/SOH
    uint16_t soc16 = (uint16_t)socPer;
    uint16_t soh   = 100;
    uint32_t socEx = (uint32_t)(socPer * 100.0f);
    memset(data, 0, 8);
    data[0]=soc16&0xFF; data[1]=(soc16>>8)&0xFF;
    data[2]=soh&0xFF;   data[3]=(soh>>8)&0xFF;
    data[4]=socEx&0xFF; data[5]=(socEx>>8)&0xFF;
    data[6]=(socEx>>16)&0xFF; data[7]=(socEx>>24)&0xFF;
    sendFrame(0x355, data, 8);

    // 0x356 - Voltage/current/temp
    int16_t pv10   = (int16_t)(packV * 100.0f);
    int16_t curr10 = (int16_t)(getCanCurrentA() * 10.0f);
    int16_t t10    = (int16_t)(avgT  * 10.0f);
    memset(data, 0, 8);
    data[0]=pv10&0xFF;   data[1]=(pv10>>8)&0xFF;
    data[2]=curr10&0xFF; data[3]=(curr10>>8)&0xFF;
    data[4]=t10&0xFF;    data[5]=(t10>>8)&0xFF;
    sendFrame(0x356, data, 8);

    // 0x35A - Alarms
    memset(data, 0, 8);
    if (faulted) {
        bool anyOV=false, anyUV=false, anyOT=false;
        for (int i=1; i<=MAX_MODULE_ADDR; i++) {
            if (!bms.getModule(i).isExisting()) continue;
            uint8_t f = bms.getModule(i).getFaults();
            if (f & 0x01) anyOV=true;
            if (f & 0x02) anyUV=true;
            if (bms.getModule(i).getAlerts() & 0x03) anyOT=true;
        }
        if (anyOV) { data[0]|=0x04; data[1]|=0x04; }
        if (anyUV) { data[0]|=0x08; data[1]|=0x08; }
        if (anyOT) { data[0]|=0x10; data[1]|=0x10; }
    }
    if (highC > settings.OverVSetpoint  - 0.05f) data[0]|=0x04;
    if (lowC  < settings.UnderVSetpoint + 0.05f) data[0]|=0x08;
    sendFrame(0x35A, data, 8);

    // 0x35E - Manufacturer
    memset(data, 0x20, 8);
    const char *mfr = "TeslaBMS";
    for (int i=0; i<8; i++) data[i]=mfr[i];
    sendFrame(0x35E, data, 8);

    // 0x35F - Chemistry/version
    memset(data, 0, 8);
    data[0]='L'; data[1]='I';
    data[2]=0x01; data[3]=0x00;
    data[4]=0x06; data[5]=0x00;  // fw version 6.0
    sendFrame(0x35F, data, 8);

    Logger::debug("CAN TX: packV=%.2fV SoC=%d%%", packV, (int)socPer);
}
