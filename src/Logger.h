#ifndef LOGGER_H_
#define LOGGER_H_
// =============================================================================
// Logger.h
// FIX: All format string arguments changed from char* to const char*
//      Eliminates all -Wwrite-strings errors/warnings throughout the project.
// =============================================================================
#include <Arduino.h>
#include "bms_config.h"

class Logger {
public:
    enum LogLevel {
        Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
    };
    static void debug(const char *fmt, ...);
    static void info(const char *fmt, ...);
    static void warn(const char *fmt, ...);
    static void error(const char *fmt, ...);
    static void console(const char *fmt, ...);
    static void setLoglevel(LogLevel);
    static LogLevel getLogLevel();
    static uint32_t getLastLogTime();
    static boolean isDebug();
private:
    static LogLevel  logLevel;
    static uint32_t  lastLogTime;
    static void log(LogLevel, const char *format, va_list);
    static void logMessage(const char *format, va_list args);
};

#endif /* LOGGER_H_ */
