#include "protocol.h"
#include "sensors.h"
#include "flightcontroller.h"
#include "comms.h"
#include <Tonttulib.h>
#include <Wire.h>
#include "transfers.h"

Tonttulib tLib;

Proto::Parser parser;
Proto::Packet pkt;

static constexpr uint32_t BRIDGE_BAUD = 921600;
static HardwareSerial& BRIDGE_SERIAL = Serial1;

static bool armed = false;
static uint8_t currentState = Proto::STATE_IDLE;
static uint16_t statusFlags = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastPrimaryMissionMs = 0;
static uint32_t lastAttitudeUs = 0;
static uint16_t txSeq = 1;

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

static void setState(uint8_t newState) {
  if (currentState == newState) return;
  currentState = newState;
  //setArmed(FcComms::stateImpliesArmed(currentState));
}

static void onSetStateCommand(uint8_t newState) {
  setState(newState);
}

static FcComms::StatusData makeStatusData() {
  FcComms::StatusData s{};
  s.state = currentState;
  s.armed = armed;
  s.gpsFix = Sensors::gpsHasFix();
  s.lipoMv = readLipoMv();

  uint16_t flags = statusFlags;
  if (s.gpsFix) flags |= Proto::STATUSF_GPS_FIX;
  else          flags &= (uint16_t)~Proto::STATUSF_GPS_FIX;
  s.flags = flags;

  return s;
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

    tLib.led.blinkFast();

    FcComms::sendBootEvent();
    FcComms::sendFcBootInfo();
    FcComms::sendStatus(makeStatusData());
    Sensors::init();
}

void loop() {

    uint32_t nowUs = micros();

    // ===== PROTOCOL =====
    //if (Proto::readPacket(Serial, parser, pkt)) {
    //    if (pkt.header.type == Proto::MSG_STATUS) {
    //        currentState = Proto::getFlightState(pkt);
    //    }
    //}

    // ===== SENSORS =====
    // KORJAUS: updateVelocity() kutsuttava jotta velX/velY päivittyvät
    Sensors::update(nowUs);
    if (Sensors::gpsHasNew()) {
        Sensors::updateVelocity(nowUs);
    }

    // ===== STATE MACHINE =====
    switch (currentState) {

        case Proto::STATE_IDLE:
            FC::disarm();
            break;

        case Proto::STATE_PRELAUNCH:
            FC::setTarget();
            if(Transfers::launchDetected(Sensors::getAccX(), Sensors::getAccY(), Sensors::getAccZ())) {
                setState(Proto::STATE_ROCKET);
            }
            break;

        case Proto::STATE_ROCKET:
            if(Transfers::deploymentDetected(Sensors::getLDR())) {
                setState(Proto::STATE_FREEFLIGHT);
            }
            break;

        case Proto::STATE_FREEFLIGHT:
            FC::arm();
            if(Transfers::stableDetected()) {
                setState(Proto::STATE_POWERED_FLIGHT);
            }
            break;

        case Proto::STATE_POWERED_FLIGHT:
            FC::update(nowUs);

            if ((uint32_t)(nowUs - lastAttitudeUs) >= 200000U) {
                lastAttitudeUs = nowUs;
                Proto::AttitudePayload a{};
                a.roll_cdeg = (int16_t)(Sensors::getRoll() * 100.0f);
                a.pitch_cdeg = (int16_t)(Sensors::getPitch() * 100.0f);
                a.yaw_cdeg = (int16_t)(Sensors::getYaw() * 100.0f);
                Proto::sendTyped(BRIDGE_SERIAL, Proto::MSG_ATTITUDE, 0, nextSeq(), a);
            } 
            if(Transfers::landingDetected(Sensors::getGroundPressure(), Sensors::getPressure())) {
                setState(Proto::STATE_GROUND);
            }
            break;
        case Proto::STATE_GROUND:
            break;
            
        default:
            FC::disarm();
            break;
    }
      tLib.update();

    FcComms::update(tLib);

    const uint32_t nowMs = millis();

    if ((uint32_t)(nowMs - lastStatusMs) >= 1000U) {
        lastStatusMs = nowMs;
        FcComms::sendStatus(makeStatusData());
    }

    if ((uint32_t)(nowMs - lastPrimaryMissionMs) >= 200U) {
        lastPrimaryMissionMs = nowMs;
        //FcComms::sendPrimaryMission(makePrimaryMissionData());
    }


}
