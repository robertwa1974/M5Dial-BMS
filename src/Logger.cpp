// =============================================================================
// Logger.cpp
// FIX: All format string arguments changed from char* to const char*
// =============================================================================
#include "Logger.h"

Logger::LogLevel Logger::logLevel    = Logger::Info;
uint32_t         Logger::lastLogTime = 0;

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
    lastLogTime = millis();
    SERIALCONSOLE.print(lastLogTime);
    SERIALCONSOLE.print(" - ");
    switch (level) {
        case Debug:  SERIALCONSOLE.print("DEBUG");   break;
        case Info:   SERIALCONSOLE.print("INFO");    break;
        case Warn:   SERIALCONSOLE.print("WARNING"); break;
        case Error:  SERIALCONSOLE.print("ERROR");   break;
        default: break;
    }
    SERIALCONSOLE.print(": ");
    logMessage(format, args);
}

void Logger::logMessage(const char *format, va_list args) {
    for (; *format != 0; ++format) {
        if (*format == '%') {
            ++format;
            if (*format == '\0') break;
            if (*format == '%')  { SERIALCONSOLE.print(*format); continue; }
            if (*format == 's')  { register char *s = (char *)va_arg(args, int); SERIALCONSOLE.print(s); continue; }
            if (*format == 'd' || *format == 'i') { SERIALCONSOLE.print(va_arg(args, int),    DEC); continue; }
            if (*format == 'f')  { SERIALCONSOLE.print(va_arg(args, double),   3);   continue; }
            if (*format == 'x')  { SERIALCONSOLE.print(va_arg(args, int),      HEX); continue; }
            if (*format == 'X')  { SERIALCONSOLE.print("0x"); SERIALCONSOLE.print(va_arg(args, int), HEX); continue; }
            if (*format == 'b')  { SERIALCONSOLE.print(va_arg(args, int),      BIN); continue; }
            if (*format == 'B')  { SERIALCONSOLE.print("0b"); SERIALCONSOLE.print(va_arg(args, int), BIN); continue; }
            if (*format == 'l')  { SERIALCONSOLE.print(va_arg(args, long),     DEC); continue; }
            if (*format == 'c')  { SERIALCONSOLE.print(va_arg(args, int));           continue; }
            if (*format == 't')  { SERIALCONSOLE.print(va_arg(args, int) == 1 ? "T" : "F");           continue; }
            if (*format == 'T')  { SERIALCONSOLE.print(va_arg(args, int) == 1 ? "TRUE" : "FALSE");    continue; }
        }
        SERIALCONSOLE.print(*format);
    }
    SERIALCONSOLE.println();
}
