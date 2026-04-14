#pragma once
// =============================================================================
// CRC8.h - CRC-8 implementation for BMW CSC CAN frames
// Polynomial : 0x1D  (SAE J1850 variant used by BMW CSC modules)
// Init value : 0xFF
// Final XOR  : caller-supplied (slot-dependent, see kCSCFinalXor table)
// Input/output reflection: none
// =============================================================================
#include <Arduino.h>

class CRC8 {
public:
    void    begin() {}   // no state to initialise
    uint8_t get_crc8(const uint8_t *data, uint8_t len, uint8_t finalxor);
};
