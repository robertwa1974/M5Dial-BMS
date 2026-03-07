#pragma once
// =============================================================================
// BMSUtil.h  - Tesla BMS UART framing layer
// Ported for M5Dial ESP32-S3:
//   - SERIAL  -> SERIALBMS  (all 5 call sites)
//   - CRC, packet framing, retry logic: unchanged
// =============================================================================
#include <Arduino.h>
#include "Logger.h"
// bms_config.h provides: SERIALBMS (extern HardwareSerial), SERIALCONSOLE

class BMSUtil {
public:

    // -------------------------------------------------------------------------
    // CRC-8 with polynomial 0x07 (CRC-8/SMBUS variant)
    // Unchanged from original - pure math, no hardware dependency.
    // -------------------------------------------------------------------------
    static uint8_t genCRC(uint8_t *input, int lenInput)
    {
        uint8_t generator = 0x07;
        uint8_t crc = 0;
        for (int x = 0; x < lenInput; x++)
        {
            crc ^= input[x];
            for (int i = 0; i < 8; i++)
            {
                if ((crc & 0x80) != 0)
                    crc = (uint8_t)((crc << 1) ^ generator);
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    // -------------------------------------------------------------------------
    // sendData - write address+command[+data][+CRC] to the CMU chain
    //
    // Teensy: SERIAL.write()  ->  ESP32-S3: SERIALBMS.write()
    // HardwareSerial::write() is non-blocking; bytes enter the TX FIFO.
    // At 612500 baud, 1 byte = ~16µs. A 4-byte write completes in ~64µs.
    // The subsequent delay(2) in callers provides adequate CMU processing time.
    // -------------------------------------------------------------------------
    static void sendData(uint8_t *data, uint8_t dataLen, bool isWrite)
    {
        uint8_t orig    = data[0];
        uint8_t addrByte = data[0];
        if (isWrite) addrByte |= 1;

        SERIALBMS.write(addrByte);
        SERIALBMS.write(&data[1], dataLen - 1);
        data[0] = addrByte;
        if (isWrite) SERIALBMS.write(genCRC(data, dataLen));

        if (Logger::isDebug())
        {
            SERIALCONSOLE.print("Sending: ");
            SERIALCONSOLE.print(addrByte, HEX);
            SERIALCONSOLE.print(" ");
            for (int x = 1; x < dataLen; x++) {
                SERIALCONSOLE.print(data[x], HEX);
                SERIALCONSOLE.print(" ");
            }
            if (isWrite) SERIALCONSOLE.print(genCRC(data, dataLen), HEX);
            SERIALCONSOLE.println();
        }

        data[0] = orig;
    }

    // -------------------------------------------------------------------------
    // getReply - wait for CMU response with timeout
    //
    // ESP32-S3 PORT FIX: The original Teensy code used a bare available()
    // check which happened to work because Teensy's UART FIFO timing aligned
    // with the fixed delays in callers. On ESP32-S3 the CMU response arrives
    // AFTER available() returns false, causing every read to return 0 bytes.
    //
    // Fix: wait up to timeoutMs for the first byte, then allow interByteMs
    // between subsequent bytes. Naturally terminates when CMU stops sending.
    // No pre-call delay() needed in callers anymore (kept for safety margin).
    // -------------------------------------------------------------------------
    static int getReply(uint8_t *data, int maxLen, int timeoutMs = 15, int interByteMs = 5)
    {
        int numBytes = 0;
        if (Logger::isDebug()) SERIALCONSOLE.print("Reply: ");

        uint32_t deadline = millis() + timeoutMs;
        while (millis() < deadline && numBytes < maxLen)
        {
            if (SERIALBMS.available())
            {
                data[numBytes] = SERIALBMS.read();
                if (Logger::isDebug()) {
                    SERIALCONSOLE.print(data[numBytes], HEX);
                    SERIALCONSOLE.print(" ");
                }
                numBytes++;
                deadline = millis() + interByteMs; // reset window after each byte
            }
        }
        while (SERIALBMS.available()) SERIALBMS.read(); // flush overrun

        if (Logger::isDebug()) SERIALCONSOLE.println();
        return numBytes;
    }

    // -------------------------------------------------------------------------
    // sendDataWithReply - send + receive with up to 3 retries
    //
    // delay(2 * ((retLen/8) + 1)):
    //   retLen=22 bytes -> delay(6ms).  At 612500 baud 22 bytes = ~288µs,
    //   so 6ms gives ample margin for CMU processing under FreeRTOS scheduler.
    //   Validated as sufficient by ronaegis ESP32-S3 port.
    // -------------------------------------------------------------------------
    static int sendDataWithReply(uint8_t *data, uint8_t dataLen, bool isWrite,
                                  uint8_t *retData, int retLen)
    {
        int attempts = 1;
        int returnedLength = 0;
        while (attempts < 4)
        {
            sendData(data, dataLen, isWrite);
            delay(2 * ((retLen / 8) + 1));
            returnedLength = getReply(retData, retLen);
            if (returnedLength == retLen) return returnedLength;
            attempts++;
        }
        return returnedLength; // caller checks vs expected length
    }
};
