// =============================================================================
// CANManager.cpp  v7
// Changes from v6:
//   - TX/RX queue sizes: tx=8 rx=32 (prevents burst overflow)
//   - Full bus-off recovery: uninstall+reinstall driver
//   - sendFrame(): state guard + non-blocking TX (drop on full queue)
//   - externalDeviceSeen: gate SimpBMS summary frames on real bus peer
//   - Skip sendI3WakeFrame() for BMWI3BUS variant
//   - New accessors: hasExternalDevice(), getI3LastSeen(), getPhevSlaveData()
//   - 0x373 extended frame: min/max cell mV + min/max temp in Kelvin (zombieverter format, unconditional)
// =============================================================================
#include "CANManager.h"
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "Logger.h"
#include "WiFiManager.h"

extern BMSModuleManager bms;
extern EEPROMSettings   settings;

// PHEV CRC finalxor table — declared extern in bms_config.h
const uint8_t BMW_PHEV_FINAL_XOR[12] = {
    0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C
};

CANManager::CANManager()
    : running(false), rxTaskHandle(nullptr), phevCmdTaskHandle(nullptr),
      lastChargerSeen(0), canCurrentA(0.0f), externalDeviceSeen(false),
      phevNextMod(0), phevMesCycle(0), phevTestCycle(0), phevBalCells(false)
{
    memset(i3data,   0, sizeof(i3data));
    memset(i3acc,    0, sizeof(i3acc));
    memset(phevdata, 0, sizeof(phevdata));
    memset(phevAcc,  0, sizeof(phevAcc));
    phevCrc8.begin();
}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------
bool CANManager::begin()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX,
        (gpio_num_t)PIN_CAN_RX,
        TWAI_MODE_NORMAL
    );
    g_config.tx_queue_len = 8;   // fits one full BMWI3BUS burst without blocking
    g_config.rx_queue_len = 32;  // headroom for multiple CSCs replying simultaneously
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

    xTaskCreatePinnedToCore(rxTaskFn, "CAN_RX", 4096, this, 5, &rxTaskHandle, 0);

    Logger::info("CAN started: TX=GPIO%d RX=GPIO%d 500kbps, RX task running",
                 PIN_CAN_TX, PIN_CAN_RX);

    // BMW i3 standard only: send wake frame.
    // Bus-pack (BMWI3BUS) does not use the 0x130 wake sequence.
    if (settings.cmuType == CMU_BMW_I3) {
        sendI3WakeFrame();
    }

    // BMW PHEV: clear stale module IDs then start poll task
    if (settings.cmuType == CMU_BMW_PHEV) {
        sendPhevResetIDs();
        vTaskDelay(pdMS_TO_TICKS(50));   // yield to scheduler before starting poll task
        xTaskCreatePinnedToCore(
            phevCmdTaskFn, "PHEV_CMD", 3072, this, 4, &phevCmdTaskHandle, 0);
        Logger::info("CAN: PHEV mode — poll task running (0x%03X..0x%03X every %dms)",
                     BMW_PHEV_CMD_BASE,
                     BMW_PHEV_CMD_BASE + BMW_PHEV_MAX_MODS - 1,
                     BMW_PHEV_CMD_RATE_MS);
    }

    return true;
}

// ---------------------------------------------------------------------------
// end()
// ---------------------------------------------------------------------------
void CANManager::end()
{
    if (!running) return;
    running = false;
    if (phevCmdTaskHandle) { vTaskDelete(phevCmdTaskHandle); phevCmdTaskHandle = nullptr; }
    if (rxTaskHandle)      { vTaskDelete(rxTaskHandle);      rxTaskHandle      = nullptr; }
    twai_stop();
    twai_driver_uninstall();
    Logger::info("CAN stopped");
}

bool CANManager::isRunning() { return running; }

// ---------------------------------------------------------------------------
// rxTaskFn - full bus-off recovery via driver uninstall+reinstall
// ---------------------------------------------------------------------------
void CANManager::rxTaskFn(void *param)
{
    CANManager *self = (CANManager *)param;
    twai_message_t msg;
    while (self->running) {
        // Check bus-off state every iteration regardless of RX activity.
        // The zombieverter floods the bus with status frames every 100ms,
        // so twai_receive() rarely times out — bus-off must be checked here.
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK &&
            status.state == TWAI_STATE_BUS_OFF) {
            Logger::warn("CAN: bus-off detected, recovering...");
            twai_stop();
            twai_driver_uninstall();
            vTaskDelay(pdMS_TO_TICKS(200));

            twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
                (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
            g.tx_queue_len = 8;
            g.rx_queue_len = 32;
            twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
            twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

            if (twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK) {
                for (int m = 0; m < I3_MAX_MODS; m++) self->i3data[m].lastSeenMs = 0;
                Logger::warn("CAN: recovered from bus-off");
            } else {
                Logger::error("CAN: bus-off recovery failed");
                self->running = false;
            }
            continue;   // restart loop after recovery
        }

        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            self->processRxFrame(msg);
        }
    }
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// phevCmdTaskFn - FreeRTOS task: send PHEV poll commands every 50ms
// ---------------------------------------------------------------------------
void CANManager::phevCmdTaskFn(void *param)
{
    CANManager *self = (CANManager *)param;
    while (self->running) {
        self->sendPhevCommand();
        vTaskDelay(pdMS_TO_TICKS(BMW_PHEV_CMD_RATE_MS));
    }
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// processRxFrame
// ---------------------------------------------------------------------------
void CANManager::processRxFrame(const twai_message_t &msg)
{
    uint32_t id  = msg.identifier;
    uint8_t  dlc = msg.data_length_code;

    if (id == settings.chargerHeartbeatID || id == 0x305 || id == 0x306) {
        lastChargerSeen    = millis();
        externalDeviceSeen = true;
        if ((id == 0x305 || id == 0x306) && dlc >= 2) {
            int16_t raw = (int16_t)((msg.data[0] << 8) | msg.data[1]);
            canCurrentA = raw * 0.1f;
        }
    }

    // BMW i3 cell voltage frames
    if (id >= BMW_I3_CELL_BASE && id <= (BMW_I3_CELL_BASE + BMW_I3_MAX_MODS - 1)) {
        int mod = (int)(id - BMW_I3_CELL_BASE) + 1;
        if (mod < 1 || mod >= I3_MAX_MODS) return;
        uint8_t sf   = (dlc >= 8) ? (msg.data[7] & 0x03) : 0;
        int     base = sf * 4;
        for (int c = 0; c < 4 && (base + c) < 12; c++) {
            uint16_t raw = ((uint16_t)msg.data[c * 2] << 8) | msg.data[c * 2 + 1];
            i3acc[mod].cells[base + c] = raw * 0.001f;
        }
        i3acc[mod].framesRx |= (1 << sf);
        if (i3acc[mod].framesRx == 0x07) {
            memcpy(i3data[mod].cellV, i3acc[mod].cells, sizeof(i3acc[mod].cells));
            i3data[mod].fresh      = true;
            i3data[mod].lastSeenMs = millis();
            i3acc[mod].framesRx    = 0;
        }
        return;
    }

    // BMW i3 temperature frames
    if (id >= BMW_I3_TEMP_BASE && id <= (BMW_I3_TEMP_BASE + BMW_I3_MAX_MODS - 1)) {
        int mod = (int)(id - BMW_I3_TEMP_BASE) + 1;
        if (mod < 1 || mod >= I3_MAX_MODS || dlc < 4) return;
        i3data[mod].temp[0] = (int16_t)((msg.data[0] << 8) | msg.data[1]) * 0.1f;
        i3data[mod].temp[1] = (int16_t)((msg.data[2] << 8) | msg.data[3]) * 0.1f;
        return;
    }

    // -----------------------------------------------------------------------
    // BMW PHEV CSC — status/error frames  (0x0A0..0x0AF)
    // type nibble 0x0 = status: bytes 0-3 error word LE, bytes 4-5 balstat LE
    // Gated on PHEV mode to avoid mis-decoding BMWI3BUS/Mini-E frames.
    // -----------------------------------------------------------------------
    if (settings.cmuType == CMU_BMW_PHEV &&
        id >= BMW_PHEV_STATUS_BASE && id <= (BMW_PHEV_STATUS_BASE + BMW_PHEV_MAX_MODS - 1))
    {
        int mod = (int)(id & 0x00F) + 1;
        if (mod < 1 || mod > BMW_PHEV_MAX_MODS || dlc < 6) {
            wifiLogCAN(id, (uint8_t *)msg.data, dlc);
            return;
        }
        phevdata[mod].error   = (uint32_t)( msg.data[0]
                              | ((uint32_t)msg.data[1] << 8)
                              | ((uint32_t)msg.data[2] << 16)
                              | ((uint32_t)msg.data[3] << 24));
        phevdata[mod].balstat = (uint16_t)(msg.data[4] | ((uint16_t)msg.data[5] << 8));
        wifiLogCAN(id, (uint8_t *)msg.data, dlc);
        return;
    }

    // -----------------------------------------------------------------------
    // BMW PHEV CSC — cell voltage frames  (0x120..0x17F)
    //
    // Frame ID = (type_nibble << 4) | mod_index  (mod_index 0-based)
    //   type 0x02 → cells  0, 1, 2   sfBit 0
    //   type 0x03 → cells  3, 4, 5   sfBit 1
    //   type 0x04 → cells  6, 7, 8   sfBit 2
    //   type 0x05 → cells  9,10,11   sfBit 3
    //   type 0x06 → cells 12,13,14   sfBit 4
    //   type 0x07 → cell  15 only    sfBit 5
    // Encoding: little-endian 14-bit, 1mV/bit
    //   v = float(buf[n*2] + (buf[n*2+1] & 0x3F) * 256) / 1000.0f
    // Commit when framesRx == 0x3F (all 6 sub-frames received).
    // Cell updates suppressed while balstat != 0 (module is balancing).
    // -----------------------------------------------------------------------
    if (settings.cmuType == CMU_BMW_PHEV &&
        id >= BMW_PHEV_CELL_BASE && id <= BMW_PHEV_CELL_MAX)
    {
        int subId = (int)(id & 0x0F0);
        int mod   = (int)(id & 0x00F) + 1;
        if (mod < 1 || mod > BMW_PHEV_MAX_MODS) {
            wifiLogCAN(id, (uint8_t *)msg.data, dlc);
            return;
        }

        int sfBit, base, numCells;
        switch (subId) {
            case 0x020: sfBit = 0; base =  0; numCells = 3; break;
            case 0x030: sfBit = 1; base =  3; numCells = 3; break;
            case 0x040: sfBit = 2; base =  6; numCells = 3; break;
            case 0x050: sfBit = 3; base =  9; numCells = 3; break;
            case 0x060: sfBit = 4; base = 12; numCells = 3; break;
            case 0x070: sfBit = 5; base = 15; numCells = 1; break;
            default:
                wifiLogCAN(id, (uint8_t *)msg.data, dlc);
                return;
        }

        // Skip cell updates during active balancing
        if (phevdata[mod].balstat == 0) {
            for (int c = 0; c < numCells; c++) {
                uint16_t raw = (uint16_t)msg.data[c * 2]
                             + (uint16_t)((msg.data[c * 2 + 1] & 0x3F) * 256);
                phevAcc[mod].cells[base + c] = raw * 0.001f;
            }
        }
        phevAcc[mod].framesRx |= (1 << sfBit);

        if (phevAcc[mod].framesRx == 0x3F) {
            memcpy(phevdata[mod].cellV, phevAcc[mod].cells, sizeof(phevAcc[mod].cells));
            phevdata[mod].fresh      = true;
            phevdata[mod].lastSeenMs = millis();
            phevAcc[mod].framesRx    = 0;
        }
        wifiLogCAN(id, (uint8_t *)msg.data, dlc);
        return;
    }

    // -----------------------------------------------------------------------
    // BMW PHEV CSC — temperature frames  (0x180..0x18F)
    //   temp[g] = buf[g] - 40  (uint8, -40°C offset, 4 sensors per module)
    // -----------------------------------------------------------------------
    if (settings.cmuType == CMU_BMW_PHEV &&
        id >= BMW_PHEV_TEMP_BASE && id <= BMW_PHEV_TEMP_MAX)
    {
        int mod = (int)(id & 0x00F) + 1;
        if (mod < 1 || mod > BMW_PHEV_MAX_MODS || dlc < 4) {
            wifiLogCAN(id, (uint8_t *)msg.data, dlc);
            return;
        }
        for (int g = 0; g < 4 && g < (int)dlc; g++) {
            phevdata[mod].temp[g] = (float)msg.data[g] - 40.0f;
        }
        phevdata[mod].lastSeenMs = millis();
        wifiLogCAN(id, (uint8_t *)msg.data, dlc);
        return;
    }

    // Any frame outside the BMW CSC range confirms an external device is present
    if (id < 0x080 || id > 0x1FF) externalDeviceSeen = true;

    wifiLogCAN(id, (uint8_t *)msg.data, dlc);
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------
bool CANManager::getChargerActive()
{
    if (lastChargerSeen == 0) return false;
    return (millis() - lastChargerSeen) < CHARGER_TIMEOUT_MS;
}

float    CANManager::getCanCurrentA()          { return canCurrentA;       }
bool     CANManager::hasExternalDevice() const { return externalDeviceSeen; }

uint32_t CANManager::getI3LastSeen(int addr) const {
    if (addr < 1 || addr >= I3_MAX_MODS) return 0;
    return i3data[addr].lastSeenMs;
}

bool CANManager::getI3SlaveData(int addr, I3SlaveData &out)
{
    if (addr < 1 || addr >= I3_MAX_MODS) return false;
    out = i3data[addr];
    i3data[addr].fresh = false;
    return out.lastSeenMs > 0;
}

bool CANManager::getPhevSlaveData(int slot, PhevSlaveData &out)
{
    if (slot < 1 || slot > BMW_PHEV_MAX_MODS) return false;
    out = phevdata[slot];
    phevdata[slot].fresh = false;
    return out.lastSeenMs > 0;
}

// ---------------------------------------------------------------------------
// getPhevChecksum
// CRC8 over [id_high, id_low, buf[0]..buf[5]], XOR with BMW_PHEV_FINAL_XOR[modIdx]
// modIdx is the 0-based module index (0..BMW_PHEV_MAX_MODS-1).
// ---------------------------------------------------------------------------
uint8_t CANManager::getPhevChecksum(uint32_t id, uint8_t *buf, int modIdx)
{
    uint8_t canmes[8];
    canmes[0] = (uint8_t)(id >> 8);
    canmes[1] = (uint8_t)(id & 0xFF);
    for (int i = 0; i < 6; i++) canmes[i + 2] = buf[i];
    return phevCrc8.get_crc8(canmes, 8, BMW_PHEV_FINAL_XOR[modIdx]);
}

// ---------------------------------------------------------------------------
// sendPhevCommand
// Sends one poll command to phevNextMod then advances the cycle state.
// Mirrors the Teensy BMWPhevBMS sendcommand() logic exactly.
// ---------------------------------------------------------------------------
void CANManager::sendPhevCommand()
{
    if (!running) return;

    uint8_t  buf[8] = {0};
    uint32_t id = (uint32_t)BMW_PHEV_CMD_BASE | (uint32_t)phevNextMod;

    // bytes 0-1: balance target voltage (LE) or idle sentinel
    if (phevBalCells) {
        uint16_t balTarget = (uint16_t)((bms.getLowCellVolt() * 1000.0f) + 5.0f);
        buf[0] = (uint8_t)(balTarget & 0xFF);
        buf[1] = (uint8_t)(balTarget >> 8);
    } else {
        buf[0] = 0xC7;
        buf[1] = 0x10;
    }

    // bytes 2-3: balance enable bits (zero = no balancing)
    buf[2] = 0x00;
    buf[3] = 0x00;

    // bytes 4-5: measurement type
    //   testCycle <  3: init sequence — request measurement start (0x20/0x00)
    //   testCycle >= 3: steady state  — request voltage+temp (0x40) or +balance (0x48)
    if (phevTestCycle < 3) {
        buf[4] = 0x20;
        buf[5] = 0x00;
    } else {
        buf[4] = phevBalCells ? 0x48 : 0x40;
        buf[5] = 0x01;
    }

    // byte 6: rolling counter nibble (upper 4 bits) + optional init flag
    buf[6] = (uint8_t)(phevMesCycle << 4);
    if (phevTestCycle == 2) buf[6] |= 0x04;

    // byte 7: CRC
    buf[7] = getPhevChecksum(id, buf, (int)phevNextMod);

    sendFrame(id, buf, 8);

    // Advance cycle — mirrors Teensy nextmes / mescycle / testcycle logic
    phevNextMod++;
    if (phevNextMod >= (uint8_t)BMW_PHEV_MAX_MODS) {
        phevNextMod = 0;
        phevMesCycle++;
        if (phevMesCycle > 0x0F) phevMesCycle = 0;
        if (phevTestCycle < 4) phevTestCycle++;
    }
}

// ---------------------------------------------------------------------------
// sendPhevResetIDs
// Broadcasts 0x0A0 wipe sequence to clear all module ID assignments.
// Called once at startup in PHEV mode before polling begins.
// ---------------------------------------------------------------------------
void CANManager::sendPhevResetIDs()
{
    uint8_t buf[8];
    for (int slot = 0; slot < BMW_PHEV_MAX_MODS + 1; slot++) {
        buf[0] = 0xA1;
        buf[1] = (uint8_t)slot;
        memset(buf + 2, 0xFF, 6);
        sendFrame(BMW_PHEV_MGMT_ID, buf, 8);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    // Broadcast "find unassigned" so modules announce themselves
    buf[0] = 0x37;
    memset(buf + 1, 0xFF, 7);
    sendFrame(BMW_PHEV_MGMT_ID, buf, 8);
    Logger::info("CAN: PHEV ID reset sent, discovery broadcast done");
}

void CANManager::sendI3WakeFrame()
{
    uint8_t data[8] = {0};
    sendFrame(BMW_I3_WAKE_ID, data, 8);
    Logger::info("CAN: BMW i3 wake frame sent (0x%03X)", BMW_I3_WAKE_ID);
}

// ---------------------------------------------------------------------------
// sendFrame — state guard + non-blocking TX
// ---------------------------------------------------------------------------
void CANManager::sendFrame(uint32_t id, uint8_t *data, uint8_t len)
{
    if (!running) return;

    // Guard: calling twai_transmit() during bus-off causes assert crash in ESP-IDF
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK ||
        status.state != TWAI_STATE_RUNNING) {
        Logger::debug("CAN TX skipped: TWAI not running (state=%d)", 
                      (twai_get_status_info(&status) == ESP_OK) ? status.state : -1);
        return;
    }

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = id;
    msg.extd             = 0;
    msg.data_length_code = (len <= 8) ? len : 8;
    memcpy(msg.data, data, msg.data_length_code);

    // Non-blocking: drop frame if queue full rather than blocking the loop task
    esp_err_t res = twai_transmit(&msg, 0);
    if (res != ESP_OK && Logger::isDebug())
        Logger::debug("CAN TX err 0x%02X on ID 0x%03X", res, id);
    wifiLogCAN(id, data, len);
}

// ---------------------------------------------------------------------------
// sendBatterySummary — standard SimpBMS frames + extended cell detail frames
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

    // 0x351 - Charge/discharge voltage and current limits
    int16_t cvh = (int16_t)(settings.OverVSetpoint  * ns * 10.0f);
    int16_t cvl = (int16_t)(settings.UnderVSetpoint * ns * 10.0f);
    int16_t ccl = 500;    // 50.0A charge current limit
    int16_t dcl = 1000;   // 100.0A discharge current limit
    data[0]=cvh&0xFF; data[1]=(cvh>>8)&0xFF;
    data[2]=ccl&0xFF; data[3]=(ccl>>8)&0xFF;
    data[4]=dcl&0xFF; data[5]=(dcl>>8)&0xFF;
    data[6]=cvl&0xFF; data[7]=(cvl>>8)&0xFF;
    sendFrame(0x351, data, 8);

    // 0x355 - SoC / SoH
    uint16_t soc16 = (uint16_t)socPer;
    uint16_t soh   = 100;
    uint32_t socEx = (uint32_t)(socPer * 100.0f);
    memset(data, 0, 8);
    data[0]=soc16&0xFF; data[1]=(soc16>>8)&0xFF;
    data[2]=soh&0xFF;   data[3]=(soh>>8)&0xFF;
    data[4]=socEx&0xFF; data[5]=(socEx>>8)&0xFF;
    data[6]=(socEx>>16)&0xFF; data[7]=(socEx>>24)&0xFF;
    sendFrame(0x355, data, 8);

    // 0x356 - Pack voltage / current / temperature
    int16_t pv10   = (int16_t)(packV * 100.0f);
    int16_t curr10 = (int16_t)(getCanCurrentA() * 10.0f);
    int16_t t10    = (int16_t)(avgT  * 10.0f);
    memset(data, 0, 8);
    data[0]=pv10&0xFF;   data[1]=(pv10>>8)&0xFF;
    data[2]=curr10&0xFF; data[3]=(curr10>>8)&0xFF;
    data[4]=t10&0xFF;    data[5]=(t10>>8)&0xFF;
    sendFrame(0x356, data, 8);

    // 0x35A - Alarms / faults bitmask
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

    // 0x35E - Manufacturer name
    memset(data, 0x20, 8);
    const char *mfr = "TeslaBMS";
    for (int i=0; i<8; i++) data[i]=mfr[i];
    sendFrame(0x35E, data, 8);

    // 0x35F - Chemistry / version
    memset(data, 0, 8);
    data[0]='L'; data[1]='I';
    data[2]=0x01; data[3]=0x00;
    data[4]=0x07; data[5]=0x00;   // fw version 7.0
    sendFrame(0x35F, data, 8);

    // -------------------------------------------------------------------------
    // 0x373 - Extended SimpBMS cell/temp summary (zombieverter format)
    // bytes 0-1: min cell mV  bytes 2-3: max cell mV
    // bytes 4-5: min temp K   bytes 6-7: max temp K
    // Temperature encoding: Kelvin = °C + 273  (zombieverter decodes as raw - 273)
    // Clamp temperatures to sane range before conversion — unconnected sensors
    // leave highestPackTemp=-100 and lowestPackTemp=200 as uninitialised sentinels.
    // -------------------------------------------------------------------------
    {
        uint16_t loMV  = (uint16_t)(lowC  * 1000.0f);
        uint16_t hiMV  = (uint16_t)(highC * 1000.0f);
        float hiT = bms.getHighTemperature();
        float loT = bms.getLowTemperature();
        // If no sensors updated the pack temps, fall back to avg (or 25°C)
        if (hiT < -99.0f || hiT > 199.0f) hiT = bms.getAvgTemperature();
        if (loT < -99.0f || loT > 199.0f) loT = bms.getAvgTemperature();
        if (hiT < -99.0f || hiT > 199.0f) hiT = 25.0f;
        if (loT < -99.0f || loT > 199.0f) loT = 25.0f;
        uint16_t minTK = (uint16_t)(loT + 273.0f);
        uint16_t maxTK = (uint16_t)(hiT + 273.0f);
        memset(data, 0, 8);
        data[0]=loMV&0xFF;  data[1]=(loMV>>8)&0xFF;
        data[2]=hiMV&0xFF;  data[3]=(hiMV>>8)&0xFF;
        data[4]=minTK&0xFF; data[5]=(minTK>>8)&0xFF;
        data[6]=maxTK&0xFF; data[7]=(maxTK>>8)&0xFF;
        sendFrame(0x373, data, 8);
    }

    Logger::debug("CAN TX: packV=%.2fV SoC=%d%%", packV, (int)socPer);
}
