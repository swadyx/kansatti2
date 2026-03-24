#include "flightcontroller.h"
#include "sensors.h"
#include "motors.h"
#include <math.h>

static bool armed=false;
static float base=0.5f;

// ===== GPS TARGET =====
static double targetLat = 0;
static double targetLon = 0;
static bool hasTarget = false;

// ==== VEL PID ====
static float velI_x = 0, velI_y = 0;
static float velD_x = 0, velD_y = 0;

static float posI_x = 0, posI_y = 0;

static float lastVelErrX = 0;
static float lastVelErrY = 0;

static constexpr float POS_KP = 0.8f;
static constexpr float POS_KI = 0.0f;
static constexpr float POS_KD = 0.1f;

static constexpr float VEL_KP = 0.5f;
static constexpr float VEL_KI = 0.1f;
static constexpr float VEL_KD = 0.05f;

static constexpr float MAX_VEL = 5.0f;
static constexpr float MAX_ANGLE = 15.0f;

static inline float clamp_f(float x,float lo,float hi){
    return x<lo?lo:x>hi?hi:x;
}

// ===== API =====
void FC::setTarget() {
    if (!hasTarget && Sensors::gpsHasFix()) {
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

void FC::update(uint32_t nowUs){

    float roll = Sensors::getRoll();
    float pitch = Sensors::getPitch();

    float gx = Sensors::getGyroX();
    float gy = Sensors::getGyroY();
    float gz = Sensors::getGyroZ();

    float velX = Sensors::velX();
    float velY = Sensors::velY();

    // KORJAUS: tuplamäärittely poistettu, vain yksi määrittely
    float rollAngleSp = 0;
    float pitchAngleSp = 0;

    if (hasTarget && Sensors::gpsHasFix()) {

        double dLat = (targetLat - Sensors::getLat()) * 111320.0;
        double dLon = (targetLon - Sensors::getLon()) * 111320.0;

        float velSpX = POS_KP * dLon;
        float velSpY = POS_KP * dLat;

        velSpX = clamp_f(velSpX, -MAX_VEL, MAX_VEL);
        velSpY = clamp_f(velSpY, -MAX_VEL, MAX_VEL);

        float velErrX = velSpX - Sensors::velX();
        float velErrY = velSpY - Sensors::velY();

        velI_x = clamp_f(velI_x + velErrX, -50, 50);
        velI_y = clamp_f(velI_y + velErrY, -50, 50);

        float velOutX =
            VEL_KP * velErrX +
            VEL_KI * velI_x;

        float velOutY =
            VEL_KP * velErrY +
            VEL_KI * velI_y;

        velOutX = clamp_f(velOutX, -MAX_ANGLE, MAX_ANGLE);
        velOutY = clamp_f(velOutY, -MAX_ANGLE, MAX_ANGLE);

        rollAngleSp  = velOutY;
        pitchAngleSp = velOutX;
    }

    // ===== ANGLE CONTROL =====
    float rollErr  = rollAngleSp - Sensors::getRoll();
    float pitchErr = pitchAngleSp - Sensors::getPitch();
    static float rollI = 0;
    static float pitchI = 0;

    rollI += rollErr;
    pitchI += pitchErr;

    float rollOut =
        1.5f * rollErr +
        0.01f * rollI -
        0.1f * Sensors::getGyroX();

    float pitchOut =
        1.5f * pitchErr +
        0.01f * pitchI -
        0.1f * Sensors::getGyroY();
    float yawOut = -Sensors::getGyroZ() * 0.1f;

    float m1 = base + rollOut + yawOut;
    float m2 = base + pitchOut - yawOut;
    float m3 = base - rollOut + yawOut;
    float m4 = base - pitchOut - yawOut;

    uint16_t u1 = constrain(m1*1000+1000,1000,2000);
    uint16_t u2 = constrain(m2*1000+1000,1000,2000);
    uint16_t u3 = constrain(m3*1000+1000,1000,2000);
    uint16_t u4 = constrain(m4*1000+1000,1000,2000);

    if (armed) Motors::set(u1,u2,u3,u4);
    else Motors::set(1000,1000,1000,1000);
}
