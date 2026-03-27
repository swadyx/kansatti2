#include "datalogger.h"
#include "sensors.h"
#include <Tonttulib.h>
#include <string.h>
#include <math.h>

extern Tonttulib tLib;

namespace DataLogger {

static constexpr uint32_t PAGE_SIZE         = 256;
static constexpr uint32_t LOG_START_PAGE    = 0;
static constexpr uint32_t LOG_START_ADDR    = LOG_START_PAGE * PAGE_SIZE;
static constexpr uint32_t LOG_PERIOD_MS     = 100;

// W25Q256
static constexpr uint32_t FLASH_TOTAL_BYTES = 32UL * 1024UL * 1024UL;
static constexpr uint32_t FLASH_TOTAL_PAGES = FLASH_TOTAL_BYTES / PAGE_SIZE;

static constexpr uint32_t RECORD_SIZE       = sizeof(TelemetryRecord);
static constexpr uint32_t RECORDS_PER_PAGE  = PAGE_SIZE / RECORD_SIZE;

static uint8_t pageBuf[PAGE_SIZE];

static uint32_t currentPage = LOG_START_PAGE;
static uint32_t writingPage = 0;
static uint8_t recordIndex = 0;

static bool ready = false;
static bool writeInProgress = false;
static Error lastError = ERR_NONE;
static uint32_t lastLogMs = 0;

static int16_t clamp_i16(long v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static uint16_t clamp_u16(long v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return (uint16_t)v;
}

static int32_t round_i32(float x) {
    return (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

static void setError(Error err) {
    lastError = err;
    ready = false;
}

static bool isValidNextWriteAddr(uint32_t addr) {
    if (addr >= FLASH_TOTAL_BYTES) return false;
    if ((addr % PAGE_SIZE) != 0)   return false;
    return true;
}

bool getStoredNextWriteAddress(uint32_t &addrOut) {
    uint32_t addr = 0;
    uint8_t gen = 0;
    uint8_t seq = 0;

    if (!tLib.eeprom.readLatestFlashAddress(addr, gen, seq)) {
        return false;
    }

    if (!isValidNextWriteAddr(addr)) {
        return false;
    }

    addrOut = addr;
    return true;
}

bool getCurrentWritePage(uint32_t &pageOut) {
    uint32_t nextAddr = 0;
    if (!getStoredNextWriteAddress(nextAddr)) {
        return false;
    }

    pageOut = nextAddr / PAGE_SIZE;
    if (pageOut >= FLASH_TOTAL_PAGES) {
        return false;
    }

    return true;
}

// static bool recoverPointer() {
//     uint32_t nextAddr = 0;

//     uint8_t gen = 0, seq = 0;
//     if (!tLib.eeprom.readLatestFlashAddress(nextAddr, gen, seq)) {
//         currentPage = LOG_START_PAGE;
//         return true;
//     }

//     if (!isValidNextWriteAddr(nextAddr)) {
//         return false;
//     }

//     currentPage = nextAddr / PAGE_SIZE;
//     if (currentPage >= FLASH_TOTAL_PAGES) {
//         return false;
//     }

//     return true;
// }

static void serviceFlash() {
    if (!writeInProgress) return;

    if (!tLib.flash.readBusyBit()) {
        const uint32_t nextAddr = (writingPage + 1U) * PAGE_SIZE;
        tLib.eeprom.writeFlashAddress(nextAddr);
        writeInProgress = false;
    }
}

static TelemetryRecord buildRecord(uint32_t nowMs, uint8_t state) {
    TelemetryRecord r{};
    r.t_ms = nowMs;

    r.ax_mg = clamp_i16((long)(Sensors::getAccX() * 1000.0f));
    r.ay_mg = clamp_i16((long)(Sensors::getAccY() * 1000.0f));
    r.az_mg = clamp_i16((long)(Sensors::getAccZ() * 1000.0f));

    r.gx_cdps = clamp_i16((long)(Sensors::getGyroX() * 100.0f));
    r.gy_cdps = clamp_i16((long)(Sensors::getGyroY() * 100.0f));
    r.gz_cdps = clamp_i16((long)(Sensors::getGyroZ() * 100.0f));

    r.pressure_pa = round_i32(Sensors::getPressure());
    r.temp_cC = clamp_i16((long)(Sensors::getTemp() * 100.0f));

    if (Sensors::gpsHasFix()) {
        r.lat_e7 = (int32_t)(Sensors::getLat() * 10000000.0);
        r.lon_e7 = (int32_t)(Sensors::getLon() * 10000000.0);
        r.alt_cm = 0;
    } else {
        r.lat_e7 = 0;
        r.lon_e7 = 0;
        r.alt_cm = 0;
    }

    r.vbat_mV = clamp_u16((long)(tLib.vlipo.readVoltage() * 1000.0f));

    r.roll_cdeg  = clamp_i16((long)(Sensors::getRoll() * 100.0f));
    r.pitch_cdeg = clamp_i16((long)(Sensors::getPitch() * 100.0f));
    r.yaw_cdeg   = clamp_i16((long)(Sensors::getYaw() * 100.0f));

    r.reserved[0] = state;

    return r;
}

static void appendRecord(const TelemetryRecord& r) {
    const uint32_t offset = recordIndex * RECORD_SIZE;
    memcpy(pageBuf + offset, &r, RECORD_SIZE);
    recordIndex++;

    if (recordIndex == RECORDS_PER_PAGE) {
        if (tLib.flash.readBusyBit()) {
            setError(ERR_FLASH_BUSY);
        } else {
            if (!tLib.flash.writeEnable()) {
                setError(ERR_WRITE_ENABLE_FAILED);
            } else if (!tLib.flash.writePage(currentPage, pageBuf)) {
                setError(ERR_WRITE_PAGE_FAILED);
            } else {
                writingPage = currentPage;
                writeInProgress = true;
                currentPage++;
            }
        }

        recordIndex = 0;
        memset(pageBuf, 0xFF, PAGE_SIZE);
    }
}

bool begin() {
    ready = false;
    writeInProgress = false;
    writingPage = 0;
    currentPage = LOG_START_PAGE;
    recordIndex = 0;
    lastError = ERR_NONE;
    lastLogMs = 0;
    memset(pageBuf, 0xFF, PAGE_SIZE);
    ready = true;
    return true;
}

bool beginRecoveredOnly(bool allowStart) {
    ready = false;
    writeInProgress = false;
    writingPage = 0;
    recordIndex = 0;
    lastError = ERR_NONE;
    lastLogMs = 0;
    memset(pageBuf, 0xFF, PAGE_SIZE);

    if (!allowStart) {
        setError(ERR_NOT_ALLOWED_TO_START);
        return false;
    }

    uint32_t nextAddr = 0;
    uint8_t gen = 0;
    uint8_t seq = 0;

    if (tLib.eeprom.readLatestFlashAddress(nextAddr, gen, seq)) {
        if (!isValidNextWriteAddr(nextAddr)) {
            setError(ERR_EEPROM_POINTER_INVALID);
            return false;
        }
        currentPage = nextAddr / PAGE_SIZE;
        if (currentPage >= FLASH_TOTAL_PAGES) {
            setError(ERR_EEPROM_POINTER_INVALID);
            return false;
        }
    } else {
        currentPage = LOG_START_PAGE;
    }

    ready = true;
    return true;
}

void update(uint32_t nowMs, uint8_t state) {
    if (!ready) return;

    serviceFlash();

    if ((uint32_t)(nowMs - lastLogMs) < LOG_PERIOD_MS) return;
    lastLogMs = nowMs;

    appendRecord(buildRecord(nowMs, state));
}

bool isReady() {
    return ready;
}

bool hasError() {
    return lastError != ERR_NONE;
}

Error getError() {
    return lastError;
}

bool flashContainsData() {
    uint32_t nextAddr = 0;
    uint8_t gen = 0;
    uint8_t seq = 0;

    if (!tLib.eeprom.readLatestFlashAddress(nextAddr, gen, seq)) {
        return false;
    }

    if (!isValidNextWriteAddr(nextAddr)) {
        return false;
    }

    return nextAddr != 0;
}

} // namespace DataLogger
