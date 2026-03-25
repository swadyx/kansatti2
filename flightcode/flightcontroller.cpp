#include "flightcontroller.h"
#include "sensors.h"
#include "motors.h"
#include <math.h>

// ===================== STATE =====================
static bool armed=false;
static float baseThrust=0.5f;

// ===================== GPS =====================
static double targetLat=0, targetLon=0;
static bool hasTarget=false;

// ===================== GPS PID =====================
static float posI_x=0, posI_y=0;
static float velI_x=0, velI_y=0;

static constexpr float POS_KP=0.8f;
static constexpr float VEL_KP=0.5f;
static constexpr float VEL_KI=0.1f;

static constexpr float MAX_VEL=5.0f;
static constexpr float MAX_ANGLE=15.0f;

// ===================== PID (VANHA) =====================
static float angleI_roll=0, angleI_pitch=0;
static float rateI_roll=0, rateI_pitch=0, rateI_yaw=0;

static float rollSp=0, pitchSp=0;

static float rollRate_prev=0, pitchRate_prev=0, yawRate_prev=0;

static constexpr float ANGLE_KP=1.5f;
static constexpr float ANGLE_KI=2.0f;

static constexpr float RATE_KP=0.001f;
static constexpr float RATE_KI=0.003f;
static constexpr float RATE_KD=0.000004f;

static constexpr float RATE_I_MAX=30;
static constexpr float MAX_RATE=400.0f;

static constexpr float ROLL_OUT_MAX=0.3f;
static constexpr float PITCH_OUT_MAX=0.3f;
static constexpr float YAW_OUT_MAX=0.12f;

// timing
static uint32_t lastRateUs=0;
static uint32_t lastAngleUs=0;

static constexpr uint32_t RATE_PERIOD_US=2000;
static constexpr uint32_t ANGLE_PERIOD_US=2000;
static constexpr float DEG_TO_RAD = 0.017453292519943295769f;
// ===================== HELPERS =====================
static inline float clamp_f(float x,float lo,float hi){
    return x<lo?lo:x>hi?hi:x;
}

// ===================== API =====================
void FC::setTarget(){
    if(!hasTarget && Sensors::gpsHasFix()){
        targetLat = Sensors::getLat();
        targetLon = Sensors::getLon();
        hasTarget = true;
    }
}

void FC::init(){
    Motors::set(1000,1000,1000,1000);
}

void FC::arm(){armed=true;}
void FC::disarm(){
    armed=false;
    Motors::set(1000,1000,1000,1000);
}

// ===================== MAIN UPDATE =====================
void FC::update(uint32_t nowUs){

    float roll  = Sensors::getRoll();
    float pitch = Sensors::getPitch();

    float gx = Sensors::getGyroX();
    float gy = Sensors::getGyroY();
    float gz = Sensors::getGyroZ();

    float rollAngleSp=0;
    float pitchAngleSp=0;

    if(hasTarget && Sensors::gpsHasFix()){

        double dLat = (targetLat - Sensors::getLat()) * 111320.0;
        double dLon = (targetLon - Sensors::getLon()) * 111320.0 * cos(Sensors::getLat() * DEG_TO_RAD);

        float velSpX = clamp_f(POS_KP * dLon, -MAX_VEL, MAX_VEL);
        float velSpY = clamp_f(POS_KP * dLat, -MAX_VEL, MAX_VEL);

        float velErrX = velSpX - Sensors::velX();
        float velErrY = velSpY - Sensors::velY();

        velI_x = clamp_f(velI_x + velErrX, -50, 50);
        velI_y = clamp_f(velI_y + velErrY, -50, 50);

        float velOutX = VEL_KP * velErrX + VEL_KI * velI_x;
        float velOutY = VEL_KP * velErrY + VEL_KI * velI_y;

        pitchAngleSp = clamp_f(velOutX, -MAX_ANGLE, MAX_ANGLE);
        rollAngleSp  = clamp_f(velOutY, -MAX_ANGLE, MAX_ANGLE);
    }


    if((int32_t)(nowUs - lastAngleUs) >= (int32_t)ANGLE_PERIOD_US){
        lastAngleUs += ANGLE_PERIOD_US;

        float errRoll  = rollAngleSp - roll;
        float errPitch = pitchAngleSp - pitch;

        if(armed){
            angleI_roll  = clamp_f(angleI_roll  + errRoll  * 0.002f, -10, 10);
            angleI_pitch = clamp_f(angleI_pitch + errPitch * 0.002f, -10, 10);
        }

        rollSp  = clamp_f(ANGLE_KP*errRoll  + ANGLE_KI*angleI_roll,  -MAX_RATE, MAX_RATE);
        pitchSp = clamp_f(ANGLE_KP*errPitch + ANGLE_KI*angleI_pitch, -MAX_RATE, MAX_RATE);
    }

    if((int32_t)(nowUs - lastRateUs) >= (int32_t)RATE_PERIOD_US){
        lastRateUs += RATE_PERIOD_US;

        float rollRate  = gx;
        float pitchRate = gy;
        float yawRate   = gz;

        float errRoll  = rollSp  - rollRate;
        float errPitch = pitchSp - pitchRate;
        float errYaw   = 0 - yawRate;

        if(armed){
            rateI_roll  = clamp_f(rateI_roll  + errRoll  * 0.002f, -RATE_I_MAX, RATE_I_MAX);
            rateI_pitch = clamp_f(rateI_pitch + errPitch * 0.002f, -RATE_I_MAX, RATE_I_MAX);
            rateI_yaw   = clamp_f(rateI_yaw   + errYaw   * 0.002f, -RATE_I_MAX, RATE_I_MAX);
        }

        float dRoll  = -(rollRate  - rollRate_prev)  / 0.002f;
        float dPitch = -(pitchRate - pitchRate_prev) / 0.002f;
        float dYaw   = -(yawRate   - yawRate_prev)   / 0.002f;

        rollRate_prev  = rollRate;
        pitchRate_prev = pitchRate;
        yawRate_prev   = yawRate;

        float rollOut =
            clamp_f(RATE_KP*errRoll + RATE_KI*rateI_roll + RATE_KD*dRoll,
                    -ROLL_OUT_MAX, ROLL_OUT_MAX);

        float pitchOut =
            clamp_f(RATE_KP*errPitch + RATE_KI*rateI_pitch + RATE_KD*dPitch,
                    -PITCH_OUT_MAX, PITCH_OUT_MAX);

        float yawOut =
            clamp_f(RATE_KP*errYaw + RATE_KI*rateI_yaw + RATE_KD*dYaw,
                    -YAW_OUT_MAX, YAW_OUT_MAX);

        // mixing
        float m1 = baseThrust + rollOut + yawOut;
        float m2 = baseThrust + pitchOut - yawOut;
        float m3 = baseThrust - rollOut + yawOut;
        float m4 = baseThrust - pitchOut - yawOut;

        uint16_t u1 = constrain(m1*1000+1000,1000,2000);
        uint16_t u2 = constrain(m2*1000+1000,1000,2000);
        uint16_t u3 = constrain(m3*1000+1000,1000,2000);
        uint16_t u4 = constrain(m4*1000+1000,1000,2000);

        if(armed) Motors::set(u1,u2,u3,u4);
        else Motors::set(1000,1000,1000,1000);
    }
}
