#include "motors.h"
#include <Tonttulib.h>

static Tonttulib tLib;

void Motors::init() {
    tLib.init();
}

void Motors::setAll(uint16_t us) {
    tLib.motors.set(1, us);
    tLib.motors.set(2, us);
    tLib.motors.set(3, us);
    tLib.motors.set(4, us);
}

void Motors::set(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    tLib.motors.set(1, m1);
    tLib.motors.set(2, m2);
    tLib.motors.set(3, m3);
    tLib.motors.set(4, m4);
}