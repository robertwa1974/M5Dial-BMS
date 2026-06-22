// =============================================================================
// Logger.cpp
// =============================================================================
#include "Logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

Logger::LogLevel  Logger::logLevel    = Logger::Info;
uint32_t          Logger::lastLogTime = 0;
SemaphoreHandle_t Logger::serialMutex = nullptr;  // created in init()

void Logger::init() {
    if (!serialMutex) {
        serialMutex = xSemaphoreCreateMutex();
        if (serialMutex) {
            SERIALCONSOLE.println("[LOGGER] Mutex initialized successfully.");
        } else {
            SERIALCONSOLE.println("[LOGGER] ERROR: Mutex initialization failed!");
        }
    }
}

void Logger::debug(const char *message, ...) {
    if (logLevel > Debug) return;
    va_list args; va_start(args, message);
    Logger::log(Debug, message, args); va_end(args);
}
void Logger::info(const char *message, ...) {
    if (logLevel > Info) return;
    va_list args; va_start(args, message);
    Logger::log(Info, message, args); va_end(args);
}
void Logger::warn(const char *message, ...) {
    if (logLevel > Warn) return;
    va_list args; va_start(args, message);
    Logger::log(Warn, message, args); va_end(args);
}
void Logger::error(const char *message, ...) {
    if (logLevel > Error) return;
    va_list args; va_start(args, message);
    Logger::log(Error, message, args); va_end(args);
}
void Logger::console(const char *message, ...) {
    va_list args; va_start(args, message);
    Logger::logMessage(message, args); va_end(args);
}

void Logger::setLoglevel(LogLevel level) { logLevel = level; }
Logger::LogLevel Logger::getLogLevel()   { return logLevel;  }
uint32_t Logger::getLastLogTime()        { return lastLogTime; }
boolean  Logger::isDebug()               { return logLevel == Debug; }

void Logger::log(LogLevel level, const char *format, va_list args) {
    char msg[256];
    vsnprintf(msg, sizeof(msg), format, args);

    const char *lvl;
    switch (level) {
        case Debug:  lvl = "DEBUG";   break;
        case Info:   lvl = "INFO";    break;
        case Warn:   lvl = "WARNING"; break;
        case Error:  lvl = "ERROR";   break;
        default:     lvl = "LOG";     break;
    }

    char line[320];
    lastLogTime = millis();
    snprintf(line, sizeof(line), "%lu - %s: %s", lastLogTime, lvl, msg);

    // Single println() is atomic within HWCDC — no mid-line interleaving
    if (serialMutex && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        SERIALCONSOLE.println(line);
        xSemaphoreGive(serialMutex);
    } else {
        SERIALCONSOLE.print("[NO MUTEX] ");
        SERIALCONSOLE.println(line);  // fallback before mutex init
    }
}

void Logger::logMessage(const char *format, va_list args) {
    char buf[256];
    vsnprintf(buf, sizeof(buf), format, args);
    if (serialMutex && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        SERIALCONSOLE.println(buf);
        xSemaphoreGive(serialMutex);
    } else {
        SERIALCONSOLE.print("[NO MUTEX] ");
        SERIALCONSOLE.println(buf);
    }
}
