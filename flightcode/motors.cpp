#include "motor.h"
#include <Tonttulib.h>

static Tonttulib tLib;

void Motor::init() {
    tLib.init();
}

void Motor::setAll(uint16_t us) {
    Serial.println(".");
}

void Motor::set(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
        Serial.println(".");

}
