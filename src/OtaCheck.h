#pragma once
// =============================================================================
// OtaCheck.h - sanity check on the first bytes of a web (OTA) firmware upload
//
// The OTA upload must be the application image (.pio/build/m5dial/firmware.bin,
// published as firmware.bin), not factory.bin. factory.bin starts with the
// bootloader, which is also an ESP image (magic 0xE9) and could pass the
// framework's own checks; booting it from an app slot would fail and only the
// rollback would save the unit. So check for the application description
// that every ESP-IDF app image carries right after the first segment header.
//
// Layout (ESP-IDF esp_app_format.h):
//   0x00  image header (24 bytes): magic 0xE9 at 0, chip_id (LE16) at 12
//   0x18  first segment header (8 bytes)
//   0x20  esp_app_desc_t: magic_word 0xABCD5432 (LE32) at 0x20,
//         version[32] at 0x30, project_name[32] at 0x50
//
// Pure functions: no Arduino calls, host-tested in test/ota_check_test.cpp.
// =============================================================================
#include <stdint.h>
#include <stddef.h>

#define OTA_IMAGE_MAGIC        0xE9
#define OTA_CHIP_ID_ESP32S3    0x0009
#define OTA_APP_DESC_MAGIC     0xABCD5432UL
#define OTA_APP_DESC_OFFSET    0x20
#define OTA_HEADER_BYTES       0x70     // enough for magic, chip id, app desc

enum OtaCheckResult {
    OTA_CHECK_OK = 0,
    OTA_CHECK_TOO_SHORT,     // first chunk too small to check
    OTA_CHECK_NOT_IMAGE,     // not an ESP image at all
    OTA_CHECK_WRONG_CHIP,    // ESP image for another chip
    OTA_CHECK_NOT_APP        // ESP image without app description: factory.bin / bootloader
};

static inline OtaCheckResult otaCheckHeader(const uint8_t *d, size_t len)
{
    if (len < OTA_HEADER_BYTES) return OTA_CHECK_TOO_SHORT;
    if (d[0] != OTA_IMAGE_MAGIC) return OTA_CHECK_NOT_IMAGE;
    uint16_t chip = (uint16_t)(d[12] | (d[13] << 8));
    if (chip != OTA_CHIP_ID_ESP32S3) return OTA_CHECK_WRONG_CHIP;
    const uint8_t *m = d + OTA_APP_DESC_OFFSET;
    uint32_t magic = (uint32_t)m[0] | ((uint32_t)m[1] << 8) |
                     ((uint32_t)m[2] << 16) | ((uint32_t)m[3] << 24);
    if (magic != OTA_APP_DESC_MAGIC) return OTA_CHECK_NOT_APP;
    return OTA_CHECK_OK;
}

static inline const char *otaCheckMessage(OtaCheckResult r)
{
    switch (r) {
        case OTA_CHECK_OK:         return "ok";
        case OTA_CHECK_TOO_SHORT:  return "upload too short to check";
        case OTA_CHECK_NOT_IMAGE:  return "not an ESP32 firmware image";
        case OTA_CHECK_WRONG_CHIP: return "firmware is for a different chip (need ESP32-S3)";
        case OTA_CHECK_NOT_APP:    return "this is factory.bin or a bootloader - upload firmware.bin";
    }
    return "unknown";
}
