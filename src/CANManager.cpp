// =============================================================================
// CANManager.cpp  v7
//
// CAN RX architecture:
//   A FreeRTOS task (core 0, 4KB stack) calls twai_receive() with 100ms
//   timeout. Received frames are dispatched to processRxFrame() which:
//     1. Updates lastChargerSeen / canCurrentA for balance inhibit
//     2. Decodes BMW i3 standard CSC frames (0x3B1-0x3B8, 0x3D1-0x3D8)
//     3. Decodes BMW i3 bus-pack frames (0x100-0x1FF)
//     4. Decodes BMW Mini-E frames (0x0A0-0x17F)
//     5. Stores i3 std enumeration reply (0x4A0) for serial console assignment
//
// BMW i3 standard CAN protocol:
//   0x3D1..0x3D8  Cell voltages, one ID per CSC module (n = ID - 0x3D0)
//     3 sub-frames per scan, 4 cells each (12 total), 16-bit BE, 1 mV/bit
//   0x3B1..0x3B8  Temperatures, one ID per CSC module
//     bytes 0-1 = temp1 (signed 16-bit, 0.1degC/bit)
//     bytes 2-3 = temp2 (signed 16-bit, 0.1degC/bit)
//   0x4A0         Unassigned CSC DMC enumeration reply (8 bytes)
//
// BMW i3 bus-pack CAN protocol:
//   TX: 0x080-0x087  one frame per slot, CRC in byte 7, every 24ms
//   RX: 0x12N-0x15N  cell voltages (LE 16-bit, 1 mV/bit)
//       0x17N         temperature  (data[4] = degC + 40)
//       0x10N         heartbeat
//
// BMW Mini-E CAN protocol:
//   TX: 0x080-0x08B  one frame per nextmes index, CRC in byte 7, every 24ms
//   RX: 0x120-0x15F  cell voltages (lo + (hi & 0x3F)*256 mV)
//       0x170-0x17F  temperature  (data[0]-40 = T1, data[1]-40 = T2)
// =============================================================================
#include "CANManager.h"
#include "CRC8.h"
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "Logger.h"
#include "WiFiManager.h"

extern BMSModuleManager bms;
extern EEPROMSettings   settings;

// ---------------------------------------------------------------------------
// CRC8 finalxor table — shared by Mini-E and BMWI3BUS TX frames
// Indexed by slot 0-11 (only first 8 values are used in practice)
// ---------------------------------------------------------------------------
static const uint8_t kCSCFinalXor[12] = {
    0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C
};

// File-scope CRC8 instance — lazy-initialised on first use
static CRC8  sCRC8;
static bool  sCRC8Ready = false;
static CRC8& getCRC8() {
    if (!sCRC8Ready) { sCRC8.begin(); sCRC8Ready = true; }
    return sCRC8;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
CANManager::CANManager()
    : running(false), rxTaskHandle(nullptr),
      lastChargerSeen(0), canCurrentA(0.0f),
      miniE_nextmes(0), miniE_mescycle(0), miniE_testcycle(0),
      bmwI3Bus_counter(0),
      unassignedSeen(false)
{
    memset(i3data,        0, sizeof(i3data));
    memset(i3acc,         0, sizeof(i3acc));
    memset(unassignedDMC, 0, sizeof(unassignedDMC));
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

    // Mode-specific startup
    if (settings.cmuType == CMU_BMW_I3) {
        sendI3WakeFrame();
        // ID assignment is driven externally by serial console / web UI on demand
    }
    // CMU_BMW_I3_BUS and CMU_BMW_MINIE: command TX driven from main loop
    // CMU_TESLA: no CAN startup needed (UART path)
    // CMU_BMW_PHEV: not yet implemented

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
    if (id == settings.chargerHeartbeatID ||
        id == 0x305 || id == 0x306)
    {
        lastChargerSeen = millis();
        if ((id == 0x305 || id == 0x306) && dlc >= 2) {
            int16_t raw = (int16_t)((msg.data[0] << 8) | msg.data[1]);
            canCurrentA = raw * 0.1f;
        }
    }

    // --- BMW i3 standard CSC cell voltage frames ---
    // 0x3D1..0x3D8: one ID per module, 3 sub-frames per scan
    // Sub-frame index in byte 7 (0,1,2 -> cells 0-3, 4-7, 8-11)
    if (settings.cmuType == CMU_BMW_I3 &&
        id >= BMW_I3_CELL_BASE &&
        id <= (BMW_I3_CELL_BASE + BMW_I3_MAX_MODS - 1))
    {
        int mod = (int)(id - BMW_I3_CELL_BASE) + 1;  // 1..BMW_I3_MAX_MODS
        if (mod < 1 || mod >= I3_MAX_MODS) return;

        uint8_t sf = (dlc >= 8) ? (msg.data[7] & 0x03) : 0;
        int base = sf * 4;

        for (int c = 0; c < 4 && (base + c) < 12; c++) {
            uint16_t raw = ((uint16_t)msg.data[c * 2] << 8) | msg.data[c * 2 + 1];
            i3acc[mod].cells[base + c] = raw * 0.001f;
        }
        i3acc[mod].framesRx |= (1 << sf);

        if (i3acc[mod].framesRx == 0x07) {   // all 3 sub-frames
            memcpy(i3data[mod].cellV, i3acc[mod].cells, sizeof(i3acc[mod].cells));
            i3data[mod].fresh      = true;
            i3data[mod].lastSeenMs = millis();
            i3acc[mod].framesRx    = 0;
        }
        return;
    }

    // --- BMW i3 standard CSC temperature frames ---
    // 0x3B1..0x3B8: bytes 0-1 = temp1, bytes 2-3 = temp2 (signed 16-bit, 0.1degC)
    if (settings.cmuType == CMU_BMW_I3 &&
        id >= BMW_I3_TEMP_BASE &&
        id <= (BMW_I3_TEMP_BASE + BMW_I3_MAX_MODS - 1))
    {
        int mod = (int)(id - BMW_I3_TEMP_BASE) + 1;
        if (mod < 1 || mod >= I3_MAX_MODS || dlc < 4) return;
        int16_t t1 = (int16_t)((msg.data[0] << 8) | msg.data[1]);
        int16_t t2 = (int16_t)((msg.data[2] << 8) | msg.data[3]);
        i3data[mod].temp[0] = t1 * 0.1f;
        i3data[mod].temp[1] = t2 * 0.1f;
        return;
    }

    // --- BMW i3 standard enumeration reply (0x4A0) ---
    if (settings.cmuType == CMU_BMW_I3 && id == 0x4A0 && dlc >= 8) {
        memcpy(unassignedDMC, msg.data, 8);
        unassignedSeen = true;
        Logger::info("CAN: unassigned CSC DMC seen");
        return;
    }

    // --- BMWI3BUS cell voltage frames: 0x120+slot, 0x130+slot, 0x140+slot, 0x150+slot ---
    // type = bits[7:4], mod_addr = bits[3:0] (0-based), mod = mod_addr+1 (1-based slot)
    // Each frame: 3 cells as LE 16-bit in bytes 0-5, 1 mV/bit
    if (settings.cmuType == CMU_BMW_I3_BUS &&
        id >= BMWI3BUS_CELL_BASE && id <= BMWI3BUS_FRAME_MAX)
    {
        int mod_addr = (int)(id & 0x00F);
        int type     = (int)((id & 0x1F0) >> 4);
        int mod      = mod_addr + 1;
        if (mod < 1 || mod >= I3_MAX_MODS) return;

        // Cell frames: type 0x12-0x15, 3 cells each (LE 16-bit)
        if (type >= 0x12 && type <= 0x15 && dlc >= 6) {
            int sub  = type - 0x12;   // 0=cells1-3, 1=cells4-6, 2=cells7-9, 3=cells10-12
            int base = sub * 3;
            for (int c = 0; c < 3; c++) {
                uint16_t raw = (uint16_t)msg.data[c * 2] |
                               ((uint16_t)msg.data[c * 2 + 1] << 8);  // little-endian
                if (raw > 0 && raw < 5000)
                    i3acc[mod].cells[base + c] = raw * 0.001f;
            }
            i3acc[mod].framesRx |= (1 << sub);
            if (i3acc[mod].framesRx == 0x0F) {   // all 4 groups (bits 0-3)
                memcpy(i3data[mod].cellV, i3acc[mod].cells, sizeof(i3acc[mod].cells));
                i3data[mod].fresh      = true;
                i3data[mod].lastSeenMs = millis();
                i3acc[mod].framesRx    = 0;
            }
            return;
        }

        // Temperature: type 0x17, data[4] = temp + 40 (degC), 0-indexed
        if (type == 0x17 && dlc >= 5) {
            i3data[mod].temp[0]    = (float)msg.data[4] - 40.0f;
            i3data[mod].temp[1]    = i3data[mod].temp[0];  // single sensor, copy both
            i3data[mod].lastSeenMs = millis();
            return;
        }

        // Heartbeat: type 0x10 — keep alive timestamp
        if (type == 0x10) {
            i3data[mod].lastSeenMs = millis();
            return;
        }

        // Raw NTC ADC: type 0x16 — store in dmcBytes for diagnostics
        if (type == 0x16 && dlc >= 6) {
            memcpy(i3data[mod].dmcBytes, msg.data, 6);
            return;
        }

        return;  // other bus-variant frame types — discard
    }

    // --- Mini-E cell voltage frames: 0x0A0-0x15F ---
    // type = bits[7:4] of lower byte, mod_addr = bits[3:0] of lower byte (0-based)
    // type 2=cells1-3, 3=cells4-6, 4=cells7-9, 5=cells10-12
    // Voltage encoding: lo + (hi & 0x3F) * 256 gives millivolts
    if (settings.cmuType == CMU_BMW_MINIE &&
        id >= MINIE_CELL_BASE && id <= MINIE_CELL_MAX)
    {
        int mod_addr = (int)(id & 0x00F);
        int type     = (int)((id & 0x0F0) >> 4);
        int mod      = mod_addr + 1;
        if (mod < 1 || mod >= I3_MAX_MODS) return;

        int sub = -1;
        if (type >= 2 && type <= 5) sub = type - 2;  // 0-3

        if (sub >= 0 && dlc >= 6) {
            int base = sub * 3;
            for (int c = 0; c < 3; c++) {
                uint8_t lo = msg.data[c * 2];
                uint8_t hi = msg.data[c * 2 + 1];
                if (hi < 0x40) {
                    i3acc[mod].cells[base + c] = (float)(lo + (hi & 0x3F) * 256) / 1000.0f;
                }
            }
            i3acc[mod].framesRx |= (1 << sub);
            if (i3acc[mod].framesRx == 0x0F) {
                memcpy(i3data[mod].cellV, i3acc[mod].cells, sizeof(i3acc[mod].cells));
                i3data[mod].fresh      = true;
                i3data[mod].lastSeenMs = millis();
                i3acc[mod].framesRx    = 0;
            }
        }
        return;
    }

    // --- Mini-E temperature frames: 0x170-0x17F ---
    // data[0] - 40 = T1 (degC), data[1] - 40 = T2 (degC)
    if (settings.cmuType == CMU_BMW_MINIE &&
        id >= MINIE_TEMP_BASE && id <= MINIE_TEMP_MAX)
    {
        int mod = (int)(id & 0x00F) + 1;
        if (mod < 1 || mod >= I3_MAX_MODS || dlc < 2) return;
        i3data[mod].temp[0]    = (float)msg.data[0] - 40.0f;
        i3data[mod].temp[1]    = (float)msg.data[1] - 40.0f;
        i3data[mod].lastSeenMs = millis();
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
// Enumeration state accessors
// ---------------------------------------------------------------------------
bool           CANManager::hasUnassignedCSC() const { return unassignedSeen; }
void           CANManager::clearUnassignedFlag()     { unassignedSeen = false; }
const uint8_t* CANManager::getUnassignedDMC()  const { return unassignedDMC; }

// ---------------------------------------------------------------------------
// sendI3WakeFrame - broadcast 0x130 to wake BMW i3 standard CSC modules
// ---------------------------------------------------------------------------
void CANManager::sendI3WakeFrame()
{
    uint8_t data[8] = {0};
    sendFrame(BMW_I3_WAKE_ID, data, 8);
    Logger::info("CAN: BMW i3 wake frame sent (0x%03X)", BMW_I3_WAKE_ID);
}

// ---------------------------------------------------------------------------
// sendI3FindUnassigned - broadcast find command; unassigned CSC replies on 0x4A0
// ---------------------------------------------------------------------------
void CANManager::sendI3FindUnassigned()
{
    uint8_t data[8] = {0};
    sendFrame(BMW_I3_CMD_ID, data, 8);
    Logger::info("CAN: i3 find-unassigned sent (0x%03X)", BMW_I3_CMD_ID);
}

// ---------------------------------------------------------------------------
// sendI3AssignID - assign newID to the CSC that replied with dmcBytes
// Three-frame sequence per BMW i3 CSC enumeration protocol.
// ---------------------------------------------------------------------------
void CANManager::sendI3AssignID(uint8_t newID, const uint8_t dmcBytes[8])
{
    uint8_t data[8];

    // Frame 1: command=0x01, new ID, first 6 DMC bytes
    data[0] = 0x01;
    data[1] = newID;
    memcpy(&data[2], dmcBytes, 6);
    sendFrame(BMW_I3_CMD_ID, data, 8);
    delay(30);

    // Frame 2: command=0x02, new ID, last 2 DMC bytes
    data[0] = 0x02;
    data[1] = newID;
    memcpy(&data[2], &dmcBytes[6], 2);
    memset(&data[4], 0, 4);
    sendFrame(BMW_I3_CMD_ID, data, 8);
    delay(10);

    // Frame 3: commit/confirm
    data[0] = 0x03;
    data[1] = newID;
    memset(&data[2], 0, 6);
    sendFrame(BMW_I3_CMD_ID, data, 8);
    delay(10);

    Logger::info("CAN: i3 assign ID %d sent", newID);
}

// ---------------------------------------------------------------------------
// sendI3ResetAllIDs - reset address registers on all CSC modules 0..maxID
// ---------------------------------------------------------------------------
void CANManager::sendI3ResetAllIDs(uint8_t maxID)
{
    uint8_t data[8] = {0};
    for (uint8_t id = 0; id <= maxID; id++) {
        data[1] = id;
        sendFrame(BMW_I3_CMD_ID, data, 8);
        delay(2);
    }
    Logger::info("CAN: i3 reset all IDs 0..%d sent", maxID);
}

// ---------------------------------------------------------------------------
// sendI3BalanceReset - send balance reset frame to all CSC modules
// ---------------------------------------------------------------------------
void CANManager::sendI3BalanceReset()
{
    uint8_t data[8] = {0};
    sendFrame(BMW_I3_BAL_RESET_ID, data, 8);
    Logger::info("CAN: i3 balance reset sent (0x%03X)", BMW_I3_BAL_RESET_ID);
}

// ---------------------------------------------------------------------------
// cscChecksum - compute CRC8 for one BMW CSC TX frame
// Input: [msgId_hi, msgId_lo, buf[0]..buf[len-2]]
// (last byte of buf is the CRC placeholder — excluded from calculation)
// ---------------------------------------------------------------------------
uint8_t CANManager::cscChecksum(uint32_t msgId, const uint8_t *buf,
                                 uint8_t len, uint8_t slotIdx)
{
    uint8_t canmes[11];
    canmes[0] = (msgId >> 8) & 0xFF;
    canmes[1] =  msgId       & 0xFF;
    int meslen = len + 1;   // +2 for ID bytes, -1 for CRC byte = net +1
    for (int i = 0; i < len - 1; i++) canmes[i + 2] = buf[i];
    return getCRC8().get_crc8(canmes, meslen, kCSCFinalXor[slotIdx % 12]);
}

// ---------------------------------------------------------------------------
// sendMiniECommand - Mini-E TX command sequencer
// Called from main loop every BMW_CSC_CMD_INTERVAL_MS.
// Iterates through message indices 0x00-0x0B (nextmes), sending one frame
// per call to 0x080+nextmes. testcycle ramps to enable measurements.
// ---------------------------------------------------------------------------
void CANManager::sendMiniECommand()
{
    uint32_t msgId = BMW_CSC_CMD_BASE | miniE_nextmes;
    uint8_t  buf[8];

    buf[0] = 0x68;
    buf[1] = 0x10;
    buf[2] = 0x00;
    buf[3] = (miniE_testcycle < 3) ? 0x00 : 0x50;
    buf[4] = 0x20;
    buf[5] = 0x00;
    buf[6] = (uint8_t)(miniE_mescycle << 4);
    if (miniE_testcycle == 2) buf[6] |= 0x04;
    buf[7] = cscChecksum(msgId, buf, 8, miniE_nextmes);

    sendFrame(msgId, buf, 8);

    // Advance sequencers
    miniE_mescycle = (miniE_mescycle + 1) & 0x0F;
    miniE_nextmes++;
    if (miniE_nextmes >= 0x0C) {
        miniE_nextmes = 0;
        if (miniE_testcycle < 4) miniE_testcycle++;
    }
}

// ---------------------------------------------------------------------------
// sendBMWI3BUSCommand - BMWI3BUS keepalive burst
// Called from main loop every BMW_CSC_CMD_INTERVAL_MS.
// Sends 8 frames (slots 0-7) to 0x080-0x087.
// First 4 calls use hardcoded init D4/counter values; then steady state.
//
// Init sequence (bmwI3Bus_counter 0-3):
//   counter  D4    counter_byte
//   0        0x00  0x10
//   1        0x00  0x20
//   2        0x00  0x34   (NOTE: 0x34 not 0x30 — confirmed from SME capture)
//   3        0x10  0x40
//
// Steady state (bmwI3Bus_counter >= 4):
//   D4 = 0x50
//   counter_byte = 0x40 + ((bmwI3Bus_counter - 3) * 0x10) & 0xFF
// ---------------------------------------------------------------------------
void CANManager::sendBMWI3BUSCommand()
{
    static const uint8_t init_d4[4]      = { 0x00, 0x00, 0x00, 0x10 };
    static const uint8_t init_counter[4] = { 0x10, 0x20, 0x34, 0x40 };

    uint8_t d4, counter;
    if (bmwI3Bus_counter < 4) {
        d4      = init_d4[bmwI3Bus_counter];
        counter = init_counter[bmwI3Bus_counter];
    } else {
        d4      = 0x50;
        counter = (uint8_t)(0x40 + ((bmwI3Bus_counter - 3) * 0x10)) & 0xFF;
    }

    for (uint8_t slot = 0; slot < 8; slot++) {
        uint32_t msgId = BMW_CSC_CMD_BASE | slot;   // 0x080-0x087
        uint8_t  buf[8];
        buf[0] = 0xC7;
        buf[1] = 0x10;
        buf[2] = 0x00;
        buf[3] = d4;
        buf[4] = 0x20;
        buf[5] = 0x00;
        buf[6] = counter;
        buf[7] = cscChecksum(msgId, buf, 8, slot);
        sendFrame(msgId, buf, 8);
    }

    bmwI3Bus_counter++;
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
    data[4]=0x07; data[5]=0x00;  // fw version 7.0
    sendFrame(0x35F, data, 8);

    Logger::debug("CAN TX: packV=%.2fV SoC=%d%%", packV, (int)socPer);
}
