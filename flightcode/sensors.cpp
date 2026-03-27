#include "sensors.h"
#include <Tonttulib.h>
#include <math.h>
#include "protocol.h"

extern Tonttulib tLib;

// ===== STATE =====
static float rollEst = 0.0f;
static float pitchEst = 0.0f;
static float yawEst = 0.0f;

static float gx_dps = 0.0f;
static float gy_dps = 0.0f;
static float gz_dps = 0.0f;

static double gyroBiasX = 0.0;
static double gyroBiasY = 0.0;
static double gyroBiasZ = 0.0;

static uint32_t lastImuUs = 0;
static constexpr uint32_t IMU_PERIOD_US = 2000;
static constexpr uint32_t BARO_PERIOD_US = 20000;

static float ax = 0.0f;
static float ay = 0.0f;
static float az = 0.0f;
static float gx = 0.0f;
static float gy = 0.0f;
static float gz = 0.0f;

static float pressure = 0.0f;
static float groundPressure = 101325.0f;

static double lastLat = 0.0;
static double lastLon = 0.0;
static uint32_t lastVelUpdateUs = 0;

static double velX_filt = 0.0f;
static double velY_filt = 0.0f;

static constexpr float ALPHA = 0.2f;
static constexpr float MAX_VEL = 10.0f;

static inline float clampf_local(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static void updatePressure50Hz() {
    static uint32_t lastUpdateUs = 0;

    uint32_t now = micros();
    if ((uint32_t)(now - lastUpdateUs) >= BARO_PERIOD_US) {
        lastUpdateUs = now;

        float newPressure = tLib.baro.readPressure();
        if (!isnan(newPressure) && newPressure > 0.0f) {
            pressure = newPressure;
        }
    }
}

// ===== INIT =====
void Sensors::init(bool recoveredInFlight) {
    if (!recoveredInFlight) {
        groundPressure = setGroundPressure();
    }
}

bool Sensors::pressureValid() {
    return !isnan(pressure) && pressure > 69.0f;
}

void Sensors::update(uint32_t nowUs, uint8_t state) {
    updatePressure50Hz();

    if ((uint32_t)(nowUs - lastImuUs) < IMU_PERIOD_US) return;
    lastImuUs = nowUs;

    tLib.imu.readAll(ax, ay, az, gx, gy, gz);

    gx_dps = gx - gyroBiasX;
    gy_dps = gy - gyroBiasY;
    gz_dps = gz - gyroBiasZ;

    const float dt = 0.002f;

    float accRoll  = atan2f(ay, az) * 57.29578f;
    float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;

    // Only estimate angles in powered flight
    if (state == Proto::STATE_POWERED_FLIGHT) {
        rollEst  = 0.985f * (rollEst  + gx_dps * dt) + 0.015f * accRoll;
        pitchEst = 0.985f * (pitchEst + gy_dps * dt) + 0.015f * accPitch;
    } else {
        // Force zero outside powered flight
        rollEst = 0.0f;
        pitchEst = 0.0f;
    }

    yawEst += gz_dps * dt;
}

void Sensors::updateVelocity(uint32_t nowUs) {
    double lat = Sensors::getLat();
    double lon = Sensors::getLon();

    if (lastVelUpdateUs == 0) {
        lastLat = lat;
        lastLon = lon;
        lastVelUpdateUs = nowUs;
        return;
    }

    float dt = (nowUs - lastVelUpdateUs) * 1e-6f;
    if (dt <= 0.0f) return;

    double dLat = (lat - lastLat) * 111320.0;
    double dLon = (lon - lastLon) * 111320.0 * cos(lat * DEG_TO_RAD);

    float vx = dLon / dt;
    float vy = dLat / dt;

    velX_filt = ALPHA * vx + (1.0f - ALPHA) * velX_filt;
    velY_filt = ALPHA * vy + (1.0f - ALPHA) * velY_filt;

    velX_filt = clampf_local(velX_filt, -MAX_VEL, MAX_VEL);
    velY_filt = clampf_local(velY_filt, -MAX_VEL, MAX_VEL);

    lastLat = lat;
    lastLon = lon;
    lastVelUpdateUs = nowUs;
}

// ===== GROUND PRESSURE =====
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
        delay(20);
    }

    groundPressure = sum / NUM_SAMPLES;
    return groundPressure;
}

void Sensors::setGroundPressureTo(float p0) {
    if (!isnan(p0) && p0 > 0.0f) {
        groundPressure = p0;
    }
}

// ===== GETTERS =====
float Sensors::getRoll() { return rollEst; }
float Sensors::getPitch() { return pitchEst; }
float Sensors::getYaw() { return yawEst; }

float Sensors::getGroundPressure() { return groundPressure; }
float Sensors::getPressure() { return pressure; }

float Sensors::getGyroX() { return gx_dps; }
float Sensors::getGyroY() { return gy_dps; }
float Sensors::getGyroZ() { return gz_dps; }

float Sensors::getAccX() { return ax; }
float Sensors::getAccY() { return ay; }
float Sensors::getAccZ() { return az; }

bool Sensors::gpsHasFix() { return tLib.gps.fixType() == 1; }
bool Sensors::gpsHasNew() {
    return tLib.gps.hasNewData();
}

void Sensors::gpsConsumeNew() {
    tLib.gps.clearNewData();
}

float Sensors::getLDR() { return tLib.ldr.readVoltage(); }
float Sensors::getTemp() { return tLib.thermistor.readCelsius(); }

double Sensors::getLat() { return tLib.gps.latitude(); }
double Sensors::getLon() { return tLib.gps.longitude(); }

float Sensors::velX() { return velX_filt; }
float Sensors::velY() { return velY_filt; }
