#include "sensors.h"
#include <Tonttulib.h>
#include <math.h>

extern Tonttulib tLib;

// ===== STATE =====
static float rollEst=0, pitchEst=0, yawEst=0;
static float gx_dps=0, gy_dps=0, gz_dps=0;

static double gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;

static uint32_t lastImuUs = 0;
static constexpr uint32_t IMU_PERIOD_US = 2000;
static constexpr uint32_t BARO_PERIOD_US = 20000;

static bool angleInit = false;
float ax, ay, az, gx, gy, gz;
float pressure = 0.0f;

static double lastLat = 0;
static double lastLon = 0;

static uint32_t lastVelUpdateUs = 0;

static float velX_filt = 0;
static float velY_filt = 0;

float groundPressure;

static constexpr float ALPHA = 0.2f;
static constexpr float MAX_VEL = 10.0f;
static inline float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

void updatePressure50Hz() {
    static uint32_t lastUpdateUs = 0;

    uint32_t now = micros();
    if (now - lastUpdateUs >= BARO_PERIOD_US) {
        lastUpdateUs = now;

        float newPressure = tLib.baro.readPressure();

        // NaN guard
        if (!isnan(newPressure)) {
            pressure = newPressure;
        }
    }
}

// ===== INIT =====
void Sensors::init() {
    groundPressure = setGroundPressure();
}

// ===== UPDATE =====
void Sensors::update(uint32_t nowUs) {

    updatePressure50Hz();

    if ((int32_t)(nowUs - lastImuUs) < IMU_PERIOD_US) return;
    lastImuUs = nowUs;

    tLib.imu.readAll(ax, ay, az, gx, gy, gz);

    gx_dps = gx - gyroBiasX;
    gy_dps = gy - gyroBiasY;
    gz_dps = gz - gyroBiasZ;

    float dt = 0.002f;

    float accRoll  = atan2f(ay, az) * 57.3f;
    float accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.3f;

    if (!angleInit) {
        rollEst = accRoll;
        pitchEst = accPitch;
        angleInit = true;
    } else {
        rollEst  = 0.985f*(rollEst  + gx_dps*dt) + 0.015f*accRoll;
        pitchEst = 0.985f*(pitchEst + gy_dps*dt) + 0.015f*accPitch;
    }

    yawEst += gz_dps * dt;
}

void Sensors::updateVelocity(uint32_t nowUs) {

    // KORJAUS: lat()/lon() → getLat()/getLon()
    double lat = Sensors::getLat();
    double lon = Sensors::getLon();

    if (lastVelUpdateUs == 0) {
        lastLat = lat;
        lastLon = lon;
        lastVelUpdateUs = nowUs;
        return;
    }

    float dt = (nowUs - lastVelUpdateUs) * 1e-6f;
    if (dt <= 0) return;

    double dLat = (lat - lastLat) * 111320.0;
    double dLon = (lon - lastLon) * 111320.0 * cos(lat * DEG_TO_RAD);

    float velX = dLon / dt;
    float velY = dLat / dt;

    velX_filt = ALPHA * velX + (1.0f - ALPHA) * velX_filt;
    velY_filt = ALPHA * velY + (1.0f - ALPHA) * velY_filt;

    velX_filt = clamp(velX_filt, -MAX_VEL, MAX_VEL);
    velY_filt = clamp(velY_filt, -MAX_VEL, MAX_VEL);

    lastLat = lat;
    lastLon = lon;
    lastVelUpdateUs = nowUs;
}

// ===== GETTERS =====
float Sensors::getRoll() { return rollEst; }
float Sensors::getPitch() { return pitchEst; }
float Sensors::getYaw() { return yawEst; }

float Sensors::getGyroX() { return gx_dps; }
float Sensors::getGyroY() { return gy_dps; }
float Sensors::getGyroZ() { return gz_dps; }

float Sensors::getAccX() { return ax; }
float Sensors::getAccY() { return ay; }
float Sensors::getAccZ() { return az; }

float Sensors::setGroundPressure() {
    const int NUM_SAMPLES = 30;
    float sum = 0.0f;
    int count = 0;
    while (count < NUM_SAMPLES) {
        float p = tLib.baro.readPressure();
        if (p > 0.0f && !isnan(p)) {
            sum += p;
            count++;
        }
        delay(20);  // matches BARO_PERIOD_US (50Hz)
    }
    return sum / NUM_SAMPLES;
}
float Sensors::getGroundPressure() {return groundPressure; }
float Sensors::getPressure() { return pressure; }

bool Sensors::gpsHasFix() { return (tLib.gps.fixType() == 1 ? true : false); }
bool Sensors::gpsHasNew() { return tLib.gps.hasNewData(); }

float Sensors::getLDR() { return tLib.ldr.readVoltage(); }

double Sensors::getLat() { return tLib.gps.latitude(); }
double Sensors::getLon() { return tLib.gps.longitude(); }

float Sensors::velX() { return velX_filt; }
float Sensors::velY() { return velY_filt; }
