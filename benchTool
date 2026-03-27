// flight_data_tool.ino
//
// Standalone bench tool for saved flight data.
//
// What it does:
// - Prints EEPROM next-write pointer diagnostics
// - Can run EEPROM pointer tests
// - Detects saved flash log using EEPROM next-write address
// - Lets you ERASE the saved log
// - Lets you DUMP the saved log as CSV over Serial
//
// IMPORTANT:
// - EEPROM stores NEXT WRITE ADDRESS in bytes.
// - Reset means next write address = 0.
// - This is a blocking bench tool.

#include <Arduino.h>
#include <Tonttulib.h>
#include <string.h>

Tonttulib tLib;

// ================= Flash layout =================
static constexpr uint32_t PAGE_SIZE           = 256;
static constexpr uint32_t LOG_START_PAGE      = 0;
static constexpr uint32_t LOG_START_ADDR      = LOG_START_PAGE * PAGE_SIZE;

static constexpr uint32_t FLASH_TOTAL_BYTES   = 32UL * 1024UL * 1024UL;
static constexpr uint32_t FLASH_TOTAL_PAGES   = FLASH_TOTAL_BYTES / PAGE_SIZE;

static constexpr uint32_t SECTOR_SIZE_BYTES   = 4096;
static constexpr uint32_t PAGES_PER_SECTOR    = SECTOR_SIZE_BYTES / PAGE_SIZE;

// ================= Current logger record layout =================
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

static constexpr uint32_t RECORD_SIZE      = sizeof(TelemetryRecord);
static constexpr uint32_t RECORDS_PER_PAGE = PAGE_SIZE / RECORD_SIZE;

static uint8_t pageBuf[PAGE_SIZE];

// ================= Helpers =================

static bool isAllFF(const uint8_t* buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        if (buf[i] != 0xFF) return false;
    }
    return true;
}

static bool isErasedRecord(const TelemetryRecord& r) {
    return isAllFF(reinterpret_cast<const uint8_t*>(&r), sizeof(TelemetryRecord));
}

static String readLineBlocking() {
    while (true) {
        if (Serial.available()) {
            String s = Serial.readStringUntil('\n');
            s.trim();
            if (s.length() > 0) return s;
        }
        delay(1);
    }
}

static bool askYesNo(const __FlashStringHelper* prompt) {
    Serial.print(prompt);
    Serial.println(F(" (y/n)"));

    while (true) {
        const String s = readLineBlocking();
        if (s.equalsIgnoreCase("y") || s.equalsIgnoreCase("yes")) return true;
        if (s.equalsIgnoreCase("n") || s.equalsIgnoreCase("no"))  return false;
        Serial.println(F("Please type y or n."));
    }
}

static bool waitAckOrNack() {
    while (true) {
        const String s = readLineBlocking();
        if (s.equalsIgnoreCase("ACK"))  return true;
        if (s.equalsIgnoreCase("NACK")) return false;
        Serial.println(F("Type ACK or NACK."));
    }
}

static bool flashWaitReady(uint32_t timeoutMs = 60000) {
    const uint32_t start = millis();
    while (tLib.flash.readBusyBit()) {
        if ((uint32_t)(millis() - start) > timeoutMs) return false;
        delay(1);
    }
    return true;
}

// ================= EEPROM pointer handling =================

static bool isValidNextWriteAddr(uint32_t addr) {
    if (addr >= FLASH_TOTAL_BYTES) return false;
    if ((addr % PAGE_SIZE) != 0)   return false;
    return true;
}

static bool getNextWriteAddr(uint32_t& nextAddrOut, uint8_t& genOut, uint8_t& seqOut) {
    uint32_t addr = 0;
    uint8_t gen = 0;
    uint8_t seq = 0;

    if (!tLib.eeprom.readLatestFlashAddress(addr, gen, seq)) {
        return false;
    }

    if (!isValidNextWriteAddr(addr)) {
        Serial.println(F("ERROR: EEPROM next write address invalid."));
        Serial.print(F("addr="));
        Serial.println(addr);
        Serial.print(F("page="));
        Serial.println(addr / PAGE_SIZE);
        return false;
    }

    nextAddrOut = addr;
    genOut = gen;
    seqOut = seq;
    return true;
}

static bool getLastWrittenPage(uint32_t& lastPageOut) {
    uint32_t nextAddr = 0;
    uint8_t gen = 0;
    uint8_t seq = 0;

    if (!getNextWriteAddr(nextAddr, gen, seq)) return false;
    if (nextAddr == 0) return false;

    const uint32_t nextPage = nextAddr / PAGE_SIZE;
    if (nextPage == 0) return false;

    lastPageOut = nextPage - 1;
    return true;
}

static void resetLogPointerToStart() {
    tLib.eeprom.writeFlashAddress(LOG_START_ADDR);
    Serial.print(F("EEPROM next-write pointer reset to nextAddr="));
    Serial.print(LOG_START_ADDR);
    Serial.print(F(" page="));
    Serial.println(LOG_START_ADDR / PAGE_SIZE);
}

static bool flashContainsData() {
    uint32_t nextAddr = 0;
    uint8_t gen = 0;
    uint8_t seq = 0;

    if (!getNextWriteAddr(nextAddr, gen, seq)) return false;
    return nextAddr != 0;
}

static void printEEPROMPointerStatus() {
    Serial.println(F("=== EEPROM pointer status ==="));

    uint32_t nextAddr = 0;
    uint8_t gen = 0;
    uint8_t seq = 0;

    const bool ok = getNextWriteAddr(nextAddr, gen, seq);

    Serial.print(F("valid="));
    Serial.println(ok ? F("true") : F("false"));

    if (ok) {
        Serial.print(F("nextAddr="));
        Serial.println(nextAddr);
        Serial.print(F("page="));
        Serial.println(nextAddr / PAGE_SIZE);
        Serial.print(F("gen="));
        Serial.println(gen);
        Serial.print(F("seq="));
        Serial.println(seq);
        Serial.print(F("containsData="));
        Serial.println(nextAddr != 0 ? F("true") : F("false"));
    }

    Serial.println(F("============================"));
}

static void testEEPROMPointer() {
    Serial.println(F("=== EEPROM pointer test ==="));

    tLib.eeprom.writeFlashAddress(0);

    uint32_t addr = 0xFFFFFFFF;
    uint8_t gen = 0;
    uint8_t seq = 0;

    bool ok = getNextWriteAddr(addr, gen, seq);

    Serial.print(F("After write 0 -> ok="));
    Serial.println(ok ? F("true") : F("false"));
    if (ok) {
        Serial.print(F("nextAddr="));
        Serial.println(addr);
        Serial.print(F("page="));
        Serial.println(addr / PAGE_SIZE);
        Serial.print(F("gen="));
        Serial.println(gen);
        Serial.print(F("seq="));
        Serial.println(seq);
    }

    tLib.eeprom.writeFlashAddress(PAGE_SIZE);

    addr = 0xFFFFFFFF;
    gen = 0;
    seq = 0;
    ok = getNextWriteAddr(addr, gen, seq);

    Serial.print(F("After write 256 -> ok="));
    Serial.println(ok ? F("true") : F("false"));
    if (ok) {
        Serial.print(F("nextAddr="));
        Serial.println(addr);
        Serial.print(F("page="));
        Serial.println(addr / PAGE_SIZE);
        Serial.print(F("gen="));
        Serial.println(gen);
        Serial.print(F("seq="));
        Serial.println(seq);
    }

    Serial.println(F("Full EEPROM dump:"));
    tLib.eeprom.dumpLog();
    Serial.println(F("========================="));
}

// ================= Printing =================

static void printCsvHeader() {
    Serial.println(
        F("t_ms,state,temp_C,pressure_Pa,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
          "lat,lon,alt_m,vbat_V,roll_deg,pitch_deg,yaw_deg")
    );
}

static void printRecordCSV(const TelemetryRecord& r) {
    const uint8_t state = r.reserved[0];

    const float tempC     = r.temp_cC / 100.0f;
    const float ax_g      = r.ax_mg / 1000.0f;
    const float ay_g      = r.ay_mg / 1000.0f;
    const float az_g      = r.az_mg / 1000.0f;
    const float gx_dps    = r.gx_cdps / 100.0f;
    const float gy_dps    = r.gy_cdps / 100.0f;
    const float gz_dps    = r.gz_cdps / 100.0f;
    const double lat      = r.lat_e7 / 1e7;
    const double lon      = r.lon_e7 / 1e7;
    const double alt_m    = r.alt_cm / 100.0;
    const float vbat_V    = r.vbat_mV / 1000.0f;
    const float roll_deg  = r.roll_cdeg / 100.0f;
    const float pitch_deg = r.pitch_cdeg / 100.0f;
    const float yaw_deg   = r.yaw_cdeg / 100.0f;

    Serial.print(r.t_ms);         Serial.print(',');
    Serial.print(state);          Serial.print(',');
    Serial.print(tempC, 2);       Serial.print(',');
    Serial.print(r.pressure_pa);  Serial.print(',');
    Serial.print(ax_g, 4);        Serial.print(',');
    Serial.print(ay_g, 4);        Serial.print(',');
    Serial.print(az_g, 4);        Serial.print(',');
    Serial.print(gx_dps, 3);      Serial.print(',');
    Serial.print(gy_dps, 3);      Serial.print(',');
    Serial.print(gz_dps, 3);      Serial.print(',');
    Serial.print(lat, 7);         Serial.print(',');
    Serial.print(lon, 7);         Serial.print(',');
    Serial.print(alt_m, 2);       Serial.print(',');
    Serial.print(vbat_V, 3);      Serial.print(',');
    Serial.print(roll_deg, 2);    Serial.print(',');
    Serial.print(pitch_deg, 2);   Serial.print(',');
    Serial.println(yaw_deg, 2);
}

// ================= Actions =================

static bool eraseLogUpToPage(uint32_t lastPage) {
    const uint32_t lastSector = lastPage / PAGES_PER_SECTOR;

    Serial.print(F("Erasing sectors 0.."));
    Serial.println(lastSector);

    for (uint32_t sector = 0; sector <= lastSector; sector++) {
        if (!flashWaitReady()) {
            Serial.println(F("ERROR: timeout waiting for flash ready before erase."));
            return false;
        }

        if (!tLib.flash.writeEnable()) {
            Serial.print(F("ERROR: writeEnable failed before erasing sector "));
            Serial.println(sector);
            return false;
        }

        if (!tLib.flash.sectorErase(sector)) {
            Serial.print(F("ERROR: sectorErase failed at sector "));
            Serial.println(sector);
            return false;
        }

        if (!flashWaitReady()) {
            Serial.println(F("ERROR: timeout waiting for flash ready during erase."));
            return false;
        }

        Serial.print(F("Erased sector "));
        Serial.println(sector);
    }

    return true;
}

static void runDumpMode(uint32_t lastPage) {
    Serial.println();
    Serial.println(F("CSV dump mode."));
    Serial.println(F("After each CSV line, type ACK to continue or NACK to resend."));
    Serial.println();

    printCsvHeader();
    Serial.println();

    Serial.print(F("Dumping pages "));
    Serial.print(LOG_START_PAGE);
    Serial.print(F(".."));
    Serial.println(lastPage);

    for (uint32_t page = LOG_START_PAGE; page <= lastPage; page++) {
        if (!tLib.flash.readPage(page, pageBuf)) {
            Serial.print(F("ERROR: Failed to read page "));
            Serial.println(page);
            return;
        }

        for (uint32_t i = 0; i < RECORDS_PER_PAGE; i++) {
            TelemetryRecord r;
            memcpy(&r, &pageBuf[i * RECORD_SIZE], RECORD_SIZE);

            if (isErasedRecord(r)) {
                Serial.println(F("# End of data (erased record)"));
                Serial.println(F("# Done"));
                return;
            }

            while (true) {
                printRecordCSV(r);
                if (waitAckOrNack()) break;
                Serial.println(F("# NACK received -> resending same record"));
            }
        }
    }

    Serial.println(F("# Done (reached end page)"));
}

static void runEraseFlow() {
    uint32_t lastPage = 0;

    if (!getLastWrittenPage(lastPage)) {
        Serial.println(F("WARNING: Could not determine last written page; erasing sector 0 only."));
        lastPage = 0;
    }

    Serial.print(F("Last written page: "));
    Serial.println(lastPage);

    if (!eraseLogUpToPage(lastPage)) {
        Serial.println(F("Erase failed."));
        return;
    }

    resetLogPointerToStart();
    Serial.println(F("Erase complete."));
}

static void runDumpFlow() {
    uint32_t lastPage = 0;

    if (!getLastWrittenPage(lastPage)) {
        Serial.println(F("No valid last written page found."));
        return;
    }

    runDumpMode(lastPage);
}

static bool askEraseTripleConfirmation() {
    Serial.println();
    Serial.println(F("WARNING: This will permanently erase the saved flight data."));
    Serial.println(F("You will be asked to confirm 3 times."));

    if (!askYesNo(F("First confirmation: Do you really want to erase the saved flight data?"))) {
        Serial.println(F("Erase cancelled at confirmation 1."));
        return false;
    }

    if (!askYesNo(F("Second confirmation: This cannot be undone. Continue with erase?"))) {
        Serial.println(F("Erase cancelled at confirmation 2."));
        return false;
    }

    if (!askYesNo(F("Third confirmation: Final check. Erase all saved flight data now?"))) {
        Serial.println(F("Erase cancelled at confirmation 3."));
        return false;
    }

    return true;
}

// ================= setup / loop =================

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println();
    Serial.println(F("--- Flight data tool ---"));

    const int status = tLib.init();
    if (status != 1) {
        Serial.print(F("Initialization failed with code: "));
        Serial.println(status);
        while (true) { delay(1000); }
    }

    tLib.led.blinkFast();

    // Uncomment only when testing EEPROM ring behavior.
    // testEEPROMPointer();

    printEEPROMPointerStatus();

    if (!flashContainsData()) {
        Serial.println(F("EEPROM says no saved flight data."));
        return;
    }

    Serial.println(F("EEPROM says saved flight data exists."));

    if (askEraseTripleConfirmation()) {
        runEraseFlow();
        return;
    }

    const bool doDump = askYesNo(F("Dump saved flight data as CSV?"));
    if (doDump) {
        runDumpFlow();
        return;
    }

    Serial.println(F("No action selected."));
}

void loop() {
}
