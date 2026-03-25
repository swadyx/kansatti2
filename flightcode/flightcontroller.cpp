#include "flightcontroller.h"
#include "sensors.h"
#include "motor.h"
#include <math.h>
#include <Tonttulib.h>

// ===================== USER SETTINGS =====================
static constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
static constexpr float    THRUST_STEP         = 0.005f;
static constexpr float    THRUST_MIN          = 0.00f;
static constexpr float    THRUST_MAX          = 0.90f;

static float baseThrust  = 0.50f;
static bool  motorsArmed = false;

// ===================== CONTROL SETTINGS =====================
static constexpr uint32_t RATE_PERIOD_US      = 2000;   // 1000 Hz
static constexpr uint32_t ANGLE_PERIOD_US     = 2000;   // 250 Hz
static constexpr float    RATE_DT             = RATE_PERIOD_US  * 1e-6f;
static constexpr float    ANGLE_DT            = ANGLE_PERIOD_US * 1e-6f;
static constexpr float    ANGLE_FUSE_ALPHA    = 0.985f;

static constexpr float ANGLE_I_MAX        = 10;
static constexpr float MAX_ANGLE_RATE_DPS = 400.0f;

static float RATE_KP_ROLL       = 0.001f;
static float RATE_KP_PITCH      = 0.001f;
static float RATE_KP_YAW        = 0.015f; 

static float RATE_KI_ROLL       = 0.003f; //0,003
static float RATE_KI_PITCH      = 0.003f; //0,003
static float RATE_KI_YAW        = 0.003f;

static float RATE_KD_ROLL       = 0.000004f;
static float RATE_KD_PITCH      = 0.000004f;
static float RATE_KD_YAW        = 0.000002f;

static float ANGLE_KP_ROLL      = 1.5f;
static float ANGLE_KP_PITCH     = 1.5f;
static float ANGLE_KI_ROLL      = 2.0f; // 2
static float ANGLE_KI_PITCH     = 2.0f; //2

static constexpr float RATE_I_MAX = 30.0f;

static constexpr float GYRO_LPF_ALPHA = 0.5f;

static float rollD_f  = 0.0f;
static float pitchD_f = 0.0f;
static float yawD_f   = 0.0f;

static constexpr float D_LPF_ALPHA = 0.09f;  // ~30 Hz at 1 kHz

static constexpr float ROLL_OUT_MAX   = 0.3f;
static constexpr float PITCH_OUT_MAX  = 0.3f;
static constexpr float YAW_OUT_MAX    = 0.12f;

static constexpr float ROLL_RATE_SIGN  = +1.0f;
static constexpr float PITCH_RATE_SIGN = +1.0f;
static constexpr float YAW_RATE_SIGN   = +1.0f;

static constexpr float ACC_ROLL_SIGN   = +1.0f;
static constexpr float ACC_PITCH_SIGN  = +1.0f;

// ===================== IMU =====================
static constexpr uint32_t IMU_PERIOD_US = 2000;

bool hasTarget = false;
float targetLon, targetLat;

// ===================== PID state =====================
static float angleI_roll=0, angleI_pitch=0;
static float rateI_roll=0, rateI_pitch=0, rateI_yaw=0;
static float rollSp=0, pitchSp=0;
static float rollRate_prev=0, pitchRate_prev=0, yawRate_prev=0;

// ===================== HELPERS =====================
static inline float clamp_f(float x,float lo,float hi){
    return x<lo?lo:x>hi?hi:x;
}
// ===================== RAMP STATE =====================
static constexpr float RAMP_DURATION_S  = 0.5f;
static constexpr float RAMP_START_NORM  = 0;   // 1200 µs = (1200-1000)/1000
static bool     rampActive  = false;
static uint32_t rampStartUs = 0;

// ================= GPS MOVEMENT =================

static double measuredPosX=0, measuredPosY=0; // m
static double lastPosX=0, lastPosY=0; // m

static double velX=0, velY=0;
static uint32_t lastGpsTime=0;

// IMU velocity damping
static float velEstX=0, velEstY=0;

// targets
static float targetPositionX = 0.0f; // m
static float targetPositionY = 0.0f; // m

// tuning
static constexpr float POS_KP = 0.4f;
static constexpr float MAX_VEL = 5.0f;
        static float gx_f=0, gy_f=0, gz_f=0;

static constexpr float VEL_KP = 4.0f;
static constexpr float VEL_DAMP_K = 1.5f;
static constexpr float MAX_ANGLE_GPS = 15.0f;

static inline float rampThrust(uint32_t nowUs) {
    if (!rampActive) return baseThrust;
    float elapsed = (nowUs - rampStartUs) * 1e-6f;
    if (elapsed >= RAMP_DURATION_S) {
        rampActive = false;
        return baseThrust;
    }
    float t = elapsed / RAMP_DURATION_S;                          // 0 → 1
    return RAMP_START_NORM + t * (baseThrust - RAMP_START_NORM);  // 0.2 → baseThrust
}
// ===================== API =====================
void FC::setTarget(){
    if(!hasTarget && Sensors::gpsHasFix()){
        targetLat = Sensors::getLat();
        targetLon = Sensors::getLon();
        hasTarget = true;
    }
}
// ===================== Motors =====================
static uint16_t m1_us=1000, m2_us=1000, m3_us=1000, m4_us=1000;
static void motorsSetAll(uint16_t us) {
    tLib.motors.set(1,us); 
    tLib.motors.set(2,us);
    tLib.motors.set(3,us); 
    tLib.motors.set(4,us);
}


void FC::init(){
    //Motors::set(1000,1000,1000,1000);
}

void FC::arm(){motorsArmed=true;}
void FC::disarm(){
    motorsArmed=false;
    //Motors::set(1000,1000,1000,1000);
}

// ===================== MAIN UPDATE =====================
void FC::update(uint32_t nowUs){
    static float velSpX = 0;
    static float velSpY = 0;

    static uint32_t lastRateUs  = 0;
    static uint32_t lastAngleUs = 0;

    if (!lastRateUs)  lastRateUs  = nowUs;
    if (!lastAngleUs) lastAngleUs = nowUs;

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


    if ((int32_t)(nowUs - lastAngleUs) >= (int32_t)ANGLE_PERIOD_US) {
        lastAngleUs += ANGLE_PERIOD_US;

        float rollAngleSp_deg =
            clamp_f(
                VEL_KP * (velSpY - velY)
                - VEL_DAMP_K * velEstY,
                -MAX_ANGLE_GPS,
                MAX_ANGLE_GPS);

        float pitchAngleSp_deg =
            clamp_f(
                VEL_KP * (velSpX - velX)
                - VEL_DAMP_K * velEstX,
                -MAX_ANGLE_GPS,
                MAX_ANGLE_GPS);

        float angleErr_roll  = rollAngleSp_deg  - Sensors::getRoll();
        float angleErr_pitch = pitchAngleSp_deg - Sensors::getPitch();

        if (motorsArmed && !rampActive) {
            angleI_roll  = clamp_f(angleI_roll  + angleErr_roll  * ANGLE_DT, -ANGLE_I_MAX, ANGLE_I_MAX);
            angleI_pitch = clamp_f(angleI_pitch + angleErr_pitch * ANGLE_DT, -ANGLE_I_MAX, ANGLE_I_MAX);
        }

        rollSp  = clamp_f(ANGLE_KP_ROLL  * angleErr_roll  + ANGLE_KI_ROLL  * angleI_roll,
                        -MAX_ANGLE_RATE_DPS, MAX_ANGLE_RATE_DPS);
        pitchSp = clamp_f(ANGLE_KP_PITCH * angleErr_pitch + ANGLE_KI_PITCH * angleI_pitch,
                        -MAX_ANGLE_RATE_DPS, MAX_ANGLE_RATE_DPS);
    }

    // Rate loop
    if ((int32_t)(nowUs - lastRateUs) >= (int32_t)RATE_PERIOD_US) {
        lastRateUs += RATE_PERIOD_US;

        // --- Gyro LPF ---
        gx_f += GYRO_LPF_ALPHA*(Sensors::getGyroX()-gx_f);
        gy_f += GYRO_LPF_ALPHA*(Sensors::getGyroY()-gy_f);
        gz_f += GYRO_LPF_ALPHA*(Sensors::getGyroZ()-gz_f);

        static float rollRate=0, pitchRate=0, yawRate=0;
        rollRate  = ROLL_RATE_SIGN  * gx_f;
        pitchRate = PITCH_RATE_SIGN * gy_f;
        yawRate   = YAW_RATE_SIGN   * gz_f;

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

        // --- Rate errors ---
        static float rateErr_roll=0, rateErr_pitch=0, rateErr_yaw=0;
        rateErr_roll  = rollSp  - rollRate;
        rateErr_pitch = pitchSp - pitchRate;
        rateErr_yaw   = 0.0f    - yawRate;

        // --- Integrators (only when armed) ---
        if (motorsArmed&& !rampActive) {
            rateI_roll  = clamp_f(rateI_roll  + rateErr_roll  * RATE_DT, -RATE_I_MAX, RATE_I_MAX);
            rateI_pitch = clamp_f(rateI_pitch + rateErr_pitch * RATE_DT, -RATE_I_MAX, RATE_I_MAX);
            rateI_yaw   = clamp_f(rateI_yaw   + rateErr_yaw   * RATE_DT, -RATE_I_MAX, RATE_I_MAX);
        }

        // --- D term on measurement, filtered ---
        float rateD_roll_raw  = -(rollRate  - rollRate_prev)  / RATE_DT;
        float rateD_pitch_raw = -(pitchRate - pitchRate_prev) / RATE_DT;
        float rateD_yaw_raw   = -(yawRate   - yawRate_prev)   / RATE_DT;

        rollD_f  += D_LPF_ALPHA * (rateD_roll_raw  - rollD_f);
        pitchD_f += D_LPF_ALPHA * (rateD_pitch_raw - pitchD_f);
        yawD_f   += D_LPF_ALPHA * (rateD_yaw_raw   - yawD_f);

        rollRate_prev  = rollRate;
        pitchRate_prev = pitchRate;
        yawRate_prev   = yawRate;

        // --- PID outputs ---
        static float rollOut=0, pitchOut=0, yawOut=0;
        rollOut  = clamp_f(RATE_KP_ROLL  * rateErr_roll  + RATE_KI_ROLL  * rateI_roll  + RATE_KD_ROLL  * rollD_f,  -ROLL_OUT_MAX,  ROLL_OUT_MAX);
        pitchOut = clamp_f(RATE_KP_PITCH * rateErr_pitch + RATE_KI_PITCH * rateI_pitch + RATE_KD_PITCH * pitchD_f, -PITCH_OUT_MAX, PITCH_OUT_MAX);
        yawOut   = clamp_f(RATE_KP_YAW   * rateErr_yaw   + RATE_KI_YAW   * rateI_yaw   + RATE_KD_YAW   * yawD_f,  -YAW_OUT_MAX,   YAW_OUT_MAX);

        float thrustNow = rampThrust(nowUs);

        float m1 = thrustNow + rollOut  + yawOut;
        float m2 = thrustNow + pitchOut - yawOut;
        float m3 = thrustNow - rollOut  + yawOut;
        float m4 = thrustNow - pitchOut - yawOut;

        m1_us = constrain((int)(m1*1000+1000), 1000, 2000);
        m2_us = constrain((int)(m2*1000+1000), 1000, 2000);
        m3_us = constrain((int)(m3*1000+1000), 1000, 2000);
        m4_us = constrain((int)(m4*1000+1000), 1000, 2000);

        if (!motorsArmed) {
            motorsSetAll(1000);
        } else {
            tLib.motors.set(1, m1_us);
            tLib.motors.set(2, m2_us);
            tLib.motors.set(3, m3_us);
            tLib.motors.set(4, m4_us);
        }
    }

}
