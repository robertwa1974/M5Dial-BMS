#pragma once
// =============================================================================
// WiFiManager.h - WiFi AP + async HTTP server for web dashboard
// =============================================================================
#include <Arduino.h>

// Free function - call from CANManager::sendFrame() to populate the rolling
// CAN log buffer served at /api/can.  Safe to call even when WiFi is off.
void wifiLogCAN(uint32_t id, uint8_t *data, uint8_t len);

class WiFiManager {
public:
    WiFiManager();
    bool begin();               // start WiFi AP and HTTP server
    void end();
    bool isRunning();
    void loop();                // call from main loop for housekeeping
    String getIP();

private:
    bool running;
    String ipAddr;
};
