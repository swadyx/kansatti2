#pragma once
#include <Arduino.h>

namespace DataLogger {

#pragma pack(push, 1)
struct TelemetryRecord {
    uint32_t t_ms;

    int16_t ax_mg;
    int16_t ay_mg;
    int16_t az_mg;

    int16_t gx_cdps;
    int16_t gy_cdps;
    int16_t gz_cdps;

    int32_t pressure_pa;
    int16_t temp_cC;

    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_cm;

    uint16_t vbat_mV;

    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;

    uint8_t reserved[22];
};
#pragma pack(pop)

static_assert(sizeof(TelemetryRecord) == 64, "TelemetryRecord must be 64 bytes");

enum Error : uint8_t {
    ERR_NONE = 0,
    ERR_NOT_ALLOWED_TO_START = 1,
    ERR_EEPROM_RECOVER_FAILED = 2,
    ERR_EEPROM_POINTER_INVALID = 3,
    ERR_WRITE_ENABLE_FAILED = 4,
    ERR_WRITE_PAGE_FAILED = 5,
    ERR_FLASH_BUSY = 6
};

bool begin();
bool beginRecoveredOnly(bool allowStart);
void update(uint32_t nowMs, uint8_t state);

bool isReady();
bool hasError();
Error getError();

bool flashContainsData();

bool getStoredNextWriteAddress(uint32_t &addrOut);
bool getCurrentWritePage(uint32_t &pageOut);

} // namespace DataLogger
