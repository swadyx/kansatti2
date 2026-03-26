#include "protocol.h"
#include "sensors.h"
#include "flightcontroller.h"
#include "comms.h"
#include <Tonttulib.h>
#include <Wire.h>
#include "transfers.h"
#include "datalogger.h"

Tonttulib tLib;

static constexpr uint32_t BRIDGE_BAUD = 921600;
static HardwareSerial& BRIDGE_SERIAL = Serial1;

static uint8_t currentState = Proto::STATE_IDLE;
static uint16_t statusFlags = 0;
static uint32_t lastStatusUs = 0;
static uint32_t lastPrimaryMissionUs = 0;
static uint32_t lastAttitudeUs = 0;
static uint16_t txSeq = 1;

static uint8_t bootRecoveredState = Proto::STATE_IDLE;
static bool referenceDataStored = false;

static uint16_t nextSeq() {
    return txSeq++;
}

static uint16_t readLipoMv() {
    const float v = tLib.vlipo.readVoltage();
    if (v <= 0.0f) return 0;
    float mv = v * 1000.0f;
    if (mv < 0.0f) mv = 0.0f;
    if (mv > 65535.0f) mv = 65535.0f;
    return (uint16_t)(mv + 0.5f);
}

static int32_t roundToInt32(float x) {
    return (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

void sendGps() {
    if (!Sensors::gpsHasNew()) return;

    Proto::GpsPayload g{};
    g.lat_e7 = (int32_t)(Sensors::getLat() * 10000000.0);
    g.lon_e7 = (int32_t)(Sensors::getLon() * 10000000.0);
    g.alt_cm = 0;
    g.sats = 0;
    g.fix_type = tLib.gps.fixType();

    float vx = Sensors::velX();
    float vy = Sensors::velY();
    float speed = sqrtf(vx * vx + vy * vy);
    g.ground_speed_cms = (int32_t)(speed * 100.0f);

    Proto::sendTyped(BRIDGE_SERIAL, Proto::MSG_GPS, 0, nextSeq(), g);
}

static void sendPrimaryMissionData() {
    Proto::PrimaryMissionPayload p{};
    p.pressure_pa    = roundToInt32(Sensors::getPressure());
    p.temperature_cC = roundToInt32(Sensors::getTemp() * 100.0f);

    Proto::sendTyped(
        BRIDGE_SERIAL,
        Proto::MSG_PRIMARY_MISSION,
        0,
        nextSeq(),
        p
    );
}

static void sendPrimaryMissionDataIfDue(uint32_t nowUs, uint32_t periodUs = 200000U) {
    if ((uint32_t)(nowUs - lastPrimaryMissionUs) < periodUs) return;
    lastPrimaryMissionUs = nowUs;
    sendPrimaryMissionData();
}

static void setState(uint8_t newState) {
    if (currentState == newState) return;
    currentState = newState;
    tLib.eeprom.writeFlightState(newState);
}

static void onSetStateCommand(uint8_t newState) {
    setState(newState);
}

static FcComms::StatusData makeStatusData() {
    FcComms::StatusData s{};
    s.state = currentState;
    s.armed = FC::isArmed();
    s.gpsFix = Sensors::gpsHasFix();
    s.lipoMv = readLipoMv();

    uint16_t flags = statusFlags;

    if (s.gpsFix) flags |= Proto::STATUSF_GPS_FIX;
    else          flags &= (uint16_t)~Proto::STATUSF_GPS_FIX;

    if (FC::hasTarget) flags |= Proto::STATUSF_HAS_TARGET;
    else               flags &= (uint16_t)~Proto::STATUSF_HAS_TARGET;

    s.flags = flags;

    return s;
}

static void tryStoreReferenceData() {
    if (referenceDataStored) return;
    if (!FC::hasTarget) return;

    const int32_t lat0_e7 = (int32_t)(FC::getTargetLat() * 10000000.0f);
    const int32_t lon0_e7 = (int32_t)(FC::getTargetLon() * 10000000.0f);
    const int32_t p0_pa   = (int32_t)(Sensors::getPressure() + (Sensors::getPressure() >= 0.0f ? 0.5f : -0.5f));

    tLib.eeprom.writeReferenceData(lat0_e7, lon0_e7, p0_pa);
    referenceDataStored = true;
}

static void restoreDataFromEeprom() {
    int32_t lat0_e7 = 0;
    int32_t lon0_e7 = 0;
    int32_t p0_pa = 0;

    tLib.eeprom.readReferenceData(lat0_e7, lon0_e7, p0_pa);

    const float lat0 = lat0_e7 / 10000000.0f;
    const float lon0 = lon0_e7 / 10000000.0f;
    const float p0   = (float)p0_pa;

    FC::setStoredTarget(lat0, lon0);

    Sensors::setGroundPressureTo((float)p0);

    referenceDataStored = true;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    const int status = tLib.init();
    if (status != 1) {
        statusFlags |= Proto::STATUSF_INIT_FAILED;
    }

    FcComms::Callbacks cb{};
    cb.onSetState = onSetStateCommand;
    FcComms::begin(BRIDGE_SERIAL, BRIDGE_BAUD, cb);

    if (status != 1) {
        FcComms::sendInitError(status);
        while (true) {
            delay(1000);
        }
    }

    bootRecoveredState = tLib.eeprom.readFlightState();

    const bool recoveredInFlight =
        (bootRecoveredState == Proto::STATE_ROCKET) ||
        (bootRecoveredState == Proto::STATE_FREEFLIGHT) ||
        (bootRecoveredState == Proto::STATE_POWERED_FLIGHT);

    const bool flashHasData = DataLogger::flashContainsData();

    if (recoveredInFlight) {
        restoreDataFromEeprom();
        currentState = bootRecoveredState;

        if (!DataLogger::beginRecoveredOnly(true)) {
            statusFlags |= Proto::STATUSF_INIT_FAILED;
            currentState = Proto::STATE_FAULT;
            FcComms::sendError(10, (int32_t)DataLogger::getError());
        } else {
            FcComms::sendEvent(Proto::EVT_BOOT, (int32_t)bootRecoveredState);
        }
    } else if (flashHasData) {
        statusFlags |= Proto::STATUSF_INIT_FAILED;
        currentState = Proto::STATE_FAULT;
        FcComms::sendError(11, (int32_t)bootRecoveredState);
        
    } else {
        currentState = bootRecoveredState;
    }

    tLib.led.blinkFast();

    FcComms::sendBootEvent();
    FcComms::sendFcBootInfo();
    Sensors::init();
}

void loop() {
    uint32_t nowUs = micros();

    Sensors::update(nowUs, currentState);
    if (Sensors::gpsHasNew()) {
        Sensors::updateVelocity(nowUs);
    }

    switch (currentState) {
        case Proto::STATE_IDLE:
            FC::disarm();
            break;

        case Proto::STATE_PRELAUNCH:
            FC::setTarget();
            FC::arm();
            tryStoreReferenceData();
            if (Transfers::launchDetected(Sensors::getAccX(), Sensors::getAccY(), Sensors::getAccZ())) {
                setState(Proto::STATE_ROCKET);
            }
            break;

        case Proto::STATE_ROCKET:
            FC::arm();
            DataLogger::update(millis(), currentState);
            if (Transfers::deploymentDetected(Sensors::getLDR())) {
                setState(Proto::STATE_FREEFLIGHT);
            }
            sendPrimaryMissionDataIfDue(nowUs);
            sendGps();
            break;

        case Proto::STATE_FREEFLIGHT:
            FC::arm();
            DataLogger::update(millis(), currentState);
            if (Transfers::stableDetected()) {
                FC::beginPoweredFlight(nowUs);
                setState(Proto::STATE_POWERED_FLIGHT);
            }
            sendPrimaryMissionDataIfDue(nowUs);
            sendGps();
            break;

        case Proto::STATE_POWERED_FLIGHT:
            FC::arm();
            FC::update(nowUs);
            DataLogger::update(millis(), currentState);
            if ((uint32_t)(nowUs - lastAttitudeUs) >= 200000U) {
                lastAttitudeUs = nowUs;
                Proto::AttitudePayload a{};
                a.roll_cdeg  = (int16_t)(Sensors::getRoll() * 100.0f);
                a.pitch_cdeg = (int16_t)(Sensors::getPitch() * 100.0f);
                a.yaw_cdeg   = (int16_t)(Sensors::getYaw() * 100.0f);
                Proto::sendTyped(BRIDGE_SERIAL, Proto::MSG_ATTITUDE, 0, nextSeq(), a);
            }

            sendPrimaryMissionDataIfDue(nowUs);
            sendGps();

            if (Transfers::landingDetected(Sensors::getGroundPressure(), Sensors::getPressure())) {
                setState(Proto::STATE_GROUND);
            }
            break;

        case Proto::STATE_GROUND:
            sendGps();
            break;

        default:
            FC::disarm();
            break;
    }

    tLib.update();
    FcComms::update(tLib);

    if ((uint32_t)(nowUs - lastStatusUs) >= 1000000U) {
        lastStatusUs = nowUs;
        FcComms::sendStatus(makeStatusData());
    }
}
