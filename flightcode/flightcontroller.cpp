#include "flightcontroller.h"
#include "sensors.h"
#include "motor.h"
#include <math.h>
#include <Tonttulib.h>

// ===================== USER SETTINGS =====================
static constexpr float THRUST_MIN = 0.00f;
static constexpr float THRUST_MAX = 0.90f;

// Hold motors at 1000 us for 8 s after powered flight begins
static constexpr uint32_t POWERED_FLIGHT_DELAY_US = 8000000UL;

static float baseThrust  = 0.50f;
static bool  motorsArmed = false;

// powered-flight gating
static bool poweredFlightStarted = false;
static uint32_t poweredFlightStartUs = 0;

// ===================== CONTROL SETTINGS =====================
static constexpr uint32_t RATE_PERIOD_US      = 2000;
static constexpr uint32_t ANGLE_PERIOD_US     = 2000;
static constexpr float    RATE_DT             = RATE_PERIOD_US  * 1e-6f;
static constexpr float    ANGLE_DT            = ANGLE_PERIOD_US * 1e-6f;

static constexpr float ANGLE_I_MAX        = 10.0f;
static constexpr float MAX_ANGLE_RATE_DPS = 400.0f;

static float RATE_KP_ROLL       = 0.001f;
static float RATE_KP_PITCH      = 0.001f;
static float RATE_KP_YAW        = 0.015f; 

static float RATE_KI_ROLL       = 0.003f;
static float RATE_KI_PITCH      = 0.003f;
static float RATE_KI_YAW        = 0.003f;

static float RATE_KD_ROLL       = 0.000004f;
static float RATE_KD_PITCH      = 0.000004f;
static float RATE_KD_YAW        = 0.000002f;

static float ANGLE_KP_ROLL      = 1.5f;
static float ANGLE_KP_PITCH     = 1.5f;
static float ANGLE_KI_ROLL      = 2.0f;
static float ANGLE_KI_PITCH     = 2.0f;

static constexpr float RATE_I_MAX = 30.0f;
static constexpr float GYRO_LPF_ALPHA = 0.5f;
static constexpr float D_LPF_ALPHA = 0.09f;

static constexpr float ROLL_OUT_MAX   = 0.3f;
static constexpr float PITCH_OUT_MAX  = 0.3f;
static constexpr float YAW_OUT_MAX    = 0.12f;

static constexpr float ROLL_RATE_SIGN  = +1.0f;
static constexpr float PITCH_RATE_SIGN = +1.0f;
static constexpr float YAW_RATE_SIGN   = +1.0f;

// ===================== TARGET =====================
bool FC::hasTarget = false;
float targetLon = 0.0f;
float targetLat = 0.0f;

// ===================== PID state =====================
static float angleI_roll=0, angleI_pitch=0;
static float rateI_roll=0, rateI_pitch=0, rateI_yaw=0;
static float rollSp=0, pitchSp=0;
static float rollRate_prev=0, pitchRate_prev=0, yawRate_prev=0;

static float rollD_f  = 0.0f;
static float pitchD_f = 0.0f;
static float yawD_f   = 0.0f;

static float gx_f=0, gy_f=0, gz_f=0;

// ===================== GPS MOVEMENT =====================
static double measuredPosX=0, measuredPosY=0;
static double lastPosX=0, lastPosY=0;
static double velX=0, velY=0;
static uint32_t lastGpsTime=0;

static float velEstX=0, velEstY=0;
static float targetPositionX = 0.0f;
static float targetPositionY = 0.0f;

static constexpr float POS_KP = 0.4f;
static constexpr float MAX_VEL = 5.0f;
static constexpr float VEL_KP = 4.0f;
static constexpr float VEL_DAMP_K = 1.5f;
static constexpr float MAX_ANGLE_GPS = 15.0f;

// ===================== RAMP STATE =====================
static constexpr float RAMP_DURATION_S  = 0.5f;
static constexpr float RAMP_START_NORM  = 0.0f;
static bool     rampActive  = false;
static uint32_t rampStartUs = 0;

// ===================== HELPERS =====================
static inline float clamp_f(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static uint16_t m1_us=1000, m2_us=1000, m3_us=1000, m4_us=1000;

static void motorsSetAll(uint16_t us) {
    tLib.motors.set(1, us);
    tLib.motors.set(2, us);
    tLib.motors.set(3, us);
    tLib.motors.set(4, us);
}

static inline float rampThrust(uint32_t nowUs) {
    if (!rampActive) return baseThrust;

    float elapsed = (nowUs - rampStartUs) * 1e-6f;
    if (elapsed >= RAMP_DURATION_S) {
        rampActive = false;
        return baseThrust;
    }

    float t = elapsed / RAMP_DURATION_S;
    return RAMP_START_NORM + t * (baseThrust - RAMP_START_NORM);
}

static void resetControllers() {
    angleI_roll = 0.0f;
    angleI_pitch = 0.0f;
    rateI_roll = 0.0f;
    rateI_pitch = 0.0f;
    rateI_yaw = 0.0f;

    rollSp = 0.0f;
    pitchSp = 0.0f;

    rollRate_prev = 0.0f;
    pitchRate_prev = 0.0f;
    yawRate_prev = 0.0f;

    rollD_f = 0.0f;
    pitchD_f = 0.0f;
    yawD_f = 0.0f;

    velEstX = 0.0f;
    velEstY = 0.0f;
}

// ===================== VERTICAL VELOCITY PID =====================
static constexpr float VZ_KP = 0.08f;
static constexpr float VZ_KI = 0.02f;
static constexpr float VZ_KD = 0.01f;
static constexpr float VZ_TARGET = -5.0f;  // m/s, negative = descending
static constexpr float VZ_I_MAX  = 0.15f;

static float vzI = 0.0f;
static float vzPrev = 0.0f;

static float computeVerticalThrust(float vz, float dt) {
    float err = VZ_TARGET - vz;

    vzI = clamp_f(vzI + err * dt, -VZ_I_MAX, VZ_I_MAX);

    float d = -(vz - vzPrev) / dt;
    vzPrev = vz;

    float out = baseThrust + VZ_KP * err + VZ_KI * vzI + VZ_KD * d;
    return clamp_f(out, THRUST_MIN, THRUST_MAX);
}
// ===================== BARO VELOCITY =====================
static float vz_filt = 0.0f;
static float lastAltM = 0.0f;
static uint32_t lastBaroUs = 0;

static constexpr float VZ_LPF_ALPHA = 0.2f;

// Convert pressure to altitude (meters)
static float pressureToAlt(float hPa) {
    return 44330.0f * (1.0f - powf(hPa / Sensors::getGroundPressure(), 0.1903f));
}

static void updateVz(uint32_t nowUs) {
    float alt = pressureToAlt(Sensors::getPressure());
    if (lastBaroUs == 0) {
        lastAltM   = alt;
        lastBaroUs = nowUs;
        return;
    }
    float dt = (nowUs - lastBaroUs) * 1e-6f;
    lastBaroUs = nowUs;

    float vz_raw = (alt - lastAltM) / dt;
    lastAltM = alt;

    vz_filt += VZ_LPF_ALPHA * (vz_raw - vz_filt);
}

// ===================== API =====================
void FC::init() {
    motorsArmed = false;
    poweredFlightStarted = false;
    poweredFlightStartUs = 0;
    rampActive = false;
    rampStartUs = 0;

    resetControllers();
    motorsSetAll(1000);
}

void FC::arm() {
    motorsArmed = true;
    motorsSetAll(1000);
}

void FC::disarm() {
    motorsArmed = false;
    poweredFlightStarted = false;
    poweredFlightStartUs = 0;
    rampActive = false;
    rampStartUs = 0;

    resetControllers();
    motorsSetAll(1000);
}

bool FC::isArmed() {
    return motorsArmed;
}

void FC::beginPoweredFlight(uint32_t nowUs) {
    if (poweredFlightStarted) return;

    poweredFlightStarted = true;
    poweredFlightStartUs = nowUs;

    resetControllers();
    motorsSetAll(1000);
}

bool FC::thrustEnabled(uint32_t nowUs) {
    if (!motorsArmed) return false;
    if (!poweredFlightStarted) return false;
    return (uint32_t)(nowUs - poweredFlightStartUs) >= POWERED_FLIGHT_DELAY_US;
}

void FC::setTarget() {
    if (!hasTarget && Sensors::gpsHasFix()) {
        targetLat = Sensors::getLat();
        targetLon = Sensors::getLon();
        hasTarget = true;
    }
}

void FC::setStoredTarget(float lat, float lon) {
    targetLat = lat;
    targetLon = lon;
    hasTarget = true;
}

float FC::getTargetLat() { return targetLat; }
float FC::getTargetLon() { return targetLon; }

// ===================== MAIN UPDATE =====================
void FC::update(uint32_t nowUs) {
    static float velSpX = 0.0f;
    static float velSpY = 0.0f;

    static uint32_t lastRateUs  = 0;
    static uint32_t lastAngleUs = 0;

    if (!lastRateUs)  lastRateUs  = nowUs;
    if (!lastAngleUs) lastAngleUs = nowUs;

    updateVz(nowUs);

    if (Sensors::gpsHasNew()) {
        double lat = Sensors::getLat();
        double lon = Sensors::getLon();

        measuredPosX = (lon - targetLon) * cos(targetLat * DEG_TO_RAD) * 111320.0;
        measuredPosY = (lat - targetLat) * 111320.0;

        uint32_t nowGps = millis();
        float dtGps = (nowGps - lastGpsTime) * 0.001f;
        lastGpsTime = nowGps;

        if (dtGps > 0.001f) {
            velX = (measuredPosX - lastPosX) / dtGps;
            velY = (measuredPosY - lastPosY) / dtGps;
            lastPosX = measuredPosX;
            lastPosY = measuredPosY;
        }

        float xErr = targetPositionX - measuredPosX;
        float yErr = targetPositionY - measuredPosY;

        velSpX = clamp_f(POS_KP * xErr, -MAX_VEL, MAX_VEL);
        velSpY = clamp_f(POS_KP * yErr, -MAX_VEL, MAX_VEL);
    }

    const bool enabled = FC::thrustEnabled(nowUs);

    if ((int32_t)(nowUs - lastAngleUs) >= (int32_t)ANGLE_PERIOD_US) {
        lastAngleUs += ANGLE_PERIOD_US;

        float rollAngleSp_deg =
            clamp_f(
                VEL_KP * (velSpY - velY) - VEL_DAMP_K * velEstY,
                -MAX_ANGLE_GPS,
                MAX_ANGLE_GPS
            );

        float pitchAngleSp_deg =
            clamp_f(
                VEL_KP * (velSpX - velX) - VEL_DAMP_K * velEstX,
                -MAX_ANGLE_GPS,
                MAX_ANGLE_GPS
            );

        float angleErr_roll  = rollAngleSp_deg  - Sensors::getRoll();
        float angleErr_pitch = pitchAngleSp_deg - Sensors::getPitch();

        if (enabled && !rampActive) {
            angleI_roll  = clamp_f(angleI_roll  + angleErr_roll  * ANGLE_DT, -ANGLE_I_MAX, ANGLE_I_MAX);
            angleI_pitch = clamp_f(angleI_pitch + angleErr_pitch * ANGLE_DT, -ANGLE_I_MAX, ANGLE_I_MAX);
        }

        rollSp  = clamp_f(ANGLE_KP_ROLL  * angleErr_roll  + ANGLE_KI_ROLL  * angleI_roll,
                          -MAX_ANGLE_RATE_DPS, MAX_ANGLE_RATE_DPS);
        pitchSp = clamp_f(ANGLE_KP_PITCH * angleErr_pitch + ANGLE_KI_PITCH * angleI_pitch,
                          -MAX_ANGLE_RATE_DPS, MAX_ANGLE_RATE_DPS);
    }

    if ((int32_t)(nowUs - lastRateUs) >= (int32_t)RATE_PERIOD_US) {
        lastRateUs += RATE_PERIOD_US;

        gx_f += GYRO_LPF_ALPHA * (Sensors::getGyroX() - gx_f);
        gy_f += GYRO_LPF_ALPHA * (Sensors::getGyroY() - gy_f);
        gz_f += GYRO_LPF_ALPHA * (Sensors::getGyroZ() - gz_f);

        float rollRate  = ROLL_RATE_SIGN  * gx_f;
        float pitchRate = PITCH_RATE_SIGN * gy_f;
        float yawRate   = YAW_RATE_SIGN   * gz_f;

        float accX_world =
            Sensors::getAccX() * cos(Sensors::getPitch() * DEG_TO_RAD) +
            Sensors::getAccZ() * sin(Sensors::getPitch() * DEG_TO_RAD);

        float accY_world =
            Sensors::getAccY() * cos(Sensors::getRoll() * DEG_TO_RAD) +
            Sensors::getAccZ() * sin(Sensors::getRoll() * DEG_TO_RAD);

        velEstX += accX_world * 9.81f * RATE_DT;
        velEstY += accY_world * 9.81f * RATE_DT;

        velEstX *= 0.98f;
        velEstY *= 0.98f;

        float rateErr_roll  = rollSp  - rollRate;
        float rateErr_pitch = pitchSp - pitchRate;
        float rateErr_yaw   = 0.0f    - yawRate;

        if (enabled && !rampActive) {
            rateI_roll  = clamp_f(rateI_roll  + rateErr_roll  * RATE_DT, -RATE_I_MAX, RATE_I_MAX);
            rateI_pitch = clamp_f(rateI_pitch + rateErr_pitch * RATE_DT, -RATE_I_MAX, RATE_I_MAX);
            rateI_yaw   = clamp_f(rateI_yaw   + rateErr_yaw   * RATE_DT, -RATE_I_MAX, RATE_I_MAX);
        }

        float rateD_roll_raw  = -(rollRate  - rollRate_prev)  / RATE_DT;
        float rateD_pitch_raw = -(pitchRate - pitchRate_prev) / RATE_DT;
        float rateD_yaw_raw   = -(yawRate   - yawRate_prev)   / RATE_DT;

        rollD_f  += D_LPF_ALPHA * (rateD_roll_raw  - rollD_f);
        pitchD_f += D_LPF_ALPHA * (rateD_pitch_raw - pitchD_f);
        yawD_f   += D_LPF_ALPHA * (rateD_yaw_raw   - yawD_f);

        rollRate_prev  = rollRate;
        pitchRate_prev = pitchRate;
        yawRate_prev   = yawRate;

        float rollOut  = clamp_f(RATE_KP_ROLL  * rateErr_roll  + RATE_KI_ROLL  * rateI_roll  + RATE_KD_ROLL  * rollD_f,
                                 -ROLL_OUT_MAX, ROLL_OUT_MAX);
        float pitchOut = clamp_f(RATE_KP_PITCH * rateErr_pitch + RATE_KI_PITCH * rateI_pitch + RATE_KD_PITCH * pitchD_f,
                                 -PITCH_OUT_MAX, PITCH_OUT_MAX);
        float yawOut   = clamp_f(RATE_KP_YAW   * rateErr_yaw   + RATE_KI_YAW   * rateI_yaw   + RATE_KD_YAW   * yawD_f,
                                 -YAW_OUT_MAX, YAW_OUT_MAX);

        if (!enabled) {
            motorsSetAll(1000);
            return;
        }

        if (!rampActive) {
            rampActive = true;
            rampStartUs = nowUs;
        }

        float thrustNow = rampThrust(nowUs);
        if(!rampActive) {
            thrustNow = computeVerticalThrust(vz_filt, RATE_DT);
        }

        float m1 = thrustNow + rollOut  + yawOut;
        float m2 = thrustNow + pitchOut - yawOut;
        float m3 = thrustNow - rollOut  + yawOut;
        float m4 = thrustNow - pitchOut - yawOut;

        m1_us = constrain((int)(m1 * 1000.0f + 1000.0f), 1000, 2000);
        m2_us = constrain((int)(m2 * 1000.0f + 1000.0f), 1000, 2000);
        m3_us = constrain((int)(m3 * 1000.0f + 1000.0f), 1000, 2000);
        m4_us = constrain((int)(m4 * 1000.0f + 1000.0f), 1000, 2000);

        tLib.motors.set(1, m1_us);
        tLib.motors.set(2, m2_us);
        tLib.motors.set(3, m3_us);
        tLib.motors.set(4, m4_us);
    }
}
