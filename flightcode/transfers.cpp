#include "transfers.h"
#include <math.h>
#include <Arduino.h>

namespace Transfers {

float DEPLOYMENT_LDR_VALUE = 0.5f;

bool launchDetected(float accX, float accY, float accZ) {
    const float THRESH_G = 1.5f;
    const unsigned long HOLD_US = 50000UL;
    static unsigned long accelAboveSince = 0;

    float mag = sqrtf(accX * accX + accY * accY + accZ * accZ);

    if (mag > THRESH_G) {
        if (accelAboveSince == 0) {
            accelAboveSince = micros();
        } else {
            if (micros() - accelAboveSince >= HOLD_US) {
                accelAboveSince = 0;
                return true;
            }
        }
    } else {
        accelAboveSince = 0;
    }

    return false;
}

bool deploymentDetected(float ldr) {
    static uint32_t deployEnterUs = 0;
    const uint32_t DEPLOY_TIMEOUT_US = 15000000UL; // 15s fallback timer

    if (deployEnterUs == 0) {
        deployEnterUs = micros(); // start timer on first call
    }

    bool ldrTriggered = ldr > DEPLOYMENT_LDR_VALUE;
    bool timerTriggered = (micros() - deployEnterUs) >= DEPLOY_TIMEOUT_US;

    return ldrTriggered || timerTriggered;
}

bool stableDetected() {
    static uint32_t stableEnterUs = 0;
    const uint32_t STABLE_HOLD_US = 2000000UL; // 2s

    if (stableEnterUs == 0) {
        stableEnterUs = micros(); // start timer on first call
    }

    return (micros() - stableEnterUs) >= STABLE_HOLD_US;
}

bool landingDetected(float p0, float pCurrent) {
    const float ALT_THRESHOLD = 1.0f; // meters

    if (p0 <= 0.0f || pCurrent <= 0.0f) return false;

    float altitude =
        44330.0f * (1.0f - powf(pCurrent / p0, 0.1903f));

    return altitude <= ALT_THRESHOLD;
}

}