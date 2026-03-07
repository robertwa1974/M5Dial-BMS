#ifndef SERIALCONSOLE_H_
#define SERIALCONSOLE_H_
// =============================================================================
// SerialConsole.h - USB serial debug interface
// Ported for M5Dial ESP32-S3:
//   - #include "bms_config.h" (replaces config.h)
//   - handleConfigCmd() added back (was removed in ronaegis port, restored here
//     so VOLTLIMHI= / TEMPLIMHI= / LOGLEVEL= etc. work over USB)
// =============================================================================
#include "bms_config.h"

class SerialConsole {
public:
    SerialConsole();
    void loop();
    void printMenu();

protected:
    enum CONSOLE_STATE { STATE_ROOT_MENU };

private:
    char cmdBuffer[80];
    int  ptrBuffer;
    int  state;
    int  loopcount;
    bool cancel;

    void init();
    void serialEvent();
    void handleConsoleCmd();
    void handleShortCmd();
    void handleConfigCmd();
};

#endif /* SERIALCONSOLE_H_ */
