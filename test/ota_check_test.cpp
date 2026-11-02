// =============================================================================
// ota_check_test.cpp - host-side unit test for src/OtaCheck.h
//
//   g++ -std=c++11 -Wall -Wextra -Isrc test/ota_check_test.cpp -o ota_check_test
//   ./ota_check_test [firmware.bin [factory.bin]]
//
// With no arguments it checks synthetic headers. Given real build outputs it
// also checks that firmware.bin passes and factory.bin is rejected.
// =============================================================================
#include <stdio.h>
#include <string.h>
#include "OtaCheck.h"

static int g_fail = 0;
static int g_pass = 0;

#define CHECK_EQ(a, b) do { \
    long long _a = (long long)(a), _b = (long long)(b); \
    if (_a == _b) { g_pass++; } \
    else { g_fail++; printf("FAIL %s:%d  %s == %s  (%lld vs %lld)\n", __FILE__, __LINE__, #a, #b, _a, _b); } \
} while (0)

static void makeApp(uint8_t *b, size_t n)
{
    memset(b, 0, n);
    b[0] = 0xE9;  b[1] = 4;              // magic, segment count
    b[12] = 0x09; b[13] = 0x00;          // chip id ESP32-S3
    b[0x20] = 0x32; b[0x21] = 0x54; b[0x22] = 0xCD; b[0x23] = 0xAB;
}

static OtaCheckResult checkFile(const char *path)
{
    uint8_t buf[OTA_HEADER_BYTES];
    FILE *f = fopen(path, "rb");
    if (!f) { printf("cannot open %s\n", path); g_fail++; return OTA_CHECK_TOO_SHORT; }
    size_t n = fread(buf, 1, sizeof(buf), f);
    fclose(f);
    return otaCheckHeader(buf, n);
}

int main(int argc, char **argv)
{
    uint8_t b[256];

    makeApp(b, sizeof(b));
    CHECK_EQ(otaCheckHeader(b, sizeof(b)), OTA_CHECK_OK);
    CHECK_EQ(otaCheckHeader(b, OTA_HEADER_BYTES), OTA_CHECK_OK);
    CHECK_EQ(otaCheckHeader(b, OTA_HEADER_BYTES - 1), OTA_CHECK_TOO_SHORT);
    CHECK_EQ(otaCheckHeader(b, 0), OTA_CHECK_TOO_SHORT);

    makeApp(b, sizeof(b)); b[0] = 0x00;
    CHECK_EQ(otaCheckHeader(b, sizeof(b)), OTA_CHECK_NOT_IMAGE);

    makeApp(b, sizeof(b)); b[12] = 0x00;            // ESP32
    CHECK_EQ(otaCheckHeader(b, sizeof(b)), OTA_CHECK_WRONG_CHIP);
    makeApp(b, sizeof(b)); b[12] = 0x05;            // ESP32-C3
    CHECK_EQ(otaCheckHeader(b, sizeof(b)), OTA_CHECK_WRONG_CHIP);

    makeApp(b, sizeof(b)); b[0x23] = 0x00;          // no app description
    CHECK_EQ(otaCheckHeader(b, sizeof(b)), OTA_CHECK_NOT_APP);

    CHECK_EQ(strcmp(otaCheckMessage(OTA_CHECK_NOT_APP),
                    "this is factory.bin or a bootloader - upload firmware.bin"), 0);

    // Real build outputs, when given
    if (argc > 1) CHECK_EQ(checkFile(argv[1]), OTA_CHECK_OK);
    if (argc > 2) CHECK_EQ(checkFile(argv[2]), OTA_CHECK_NOT_APP);

    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
