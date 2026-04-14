// =============================================================================
// CRC8.cpp - CRC-8 (poly 0x1D, init 0xFF) for BMW CSC CAN frames
// =============================================================================
#include "CRC8.h"

// Bit-by-bit CRC-8 with polynomial 0x1D, initial value 0xFF.
// finalxor is XORed into the result before returning (slot-dependent value).
uint8_t CRC8::get_crc8(const uint8_t *data, uint8_t len, uint8_t finalxor)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x1D);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc ^ finalxor;
}
