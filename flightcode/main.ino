#include "protocol.h"
#include "sensors.h"
#include "flightcontroller.h"

Proto::Parser parser;
Proto::Packet pkt;
Proto::FlightState currentState = Proto::STATE_IDLE;

void setup() {
    Serial.begin(115200);
    Sensors::init();
}

void loop() {

    uint32_t nowUs = micros();

    // ===== PROTOCOL =====
    if (Proto::readPacket(Serial, parser, pkt)) {
        if (pkt.header.type == Proto::MSG_STATUS) {
            currentState = Proto::getFlightState(pkt);
        }
    }

    // ===== SENSORS =====
    Sensors::update(nowUs);

    // KORJAUS: updateVelocity() kutsuttava jotta velX/velY päivittyvät
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
            break;

        case Proto::STATE_FREEFLIGHT:
            FC::arm();
            break;

        case Proto::STATE_POWERED_FLIGHT:
            FC::update(nowUs);
            break;

        default:
            FC::disarm();
            break;
    }
}
