#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <Tonttulib.h>
#include <math.h>
#include "protocol.h"

Tonttulib tLib;

// ===================== USER SETTINGS =====================
static constexpr uint32_t TELEMETRY_PERIOD_MS = 200;   // 5 Hz

// thrust control
static constexpr float THRUST_STEP = 0.02f;  // per command
static constexpr float THRUST_MIN  = 0.00f;
static constexpr float THRUST_MAX  = 0.90f;  // safety cap

static float baseThrust = 0.55f;
static bool motorsArmed = false; // FALSE by default

// ===================== CONTROL SETTINGS =====================
// Run controller at a fixed rate.
static constexpr uint32_t CTRL_PERIOD_US = 5000; // 200 Hz

// Complementary filter
static constexpr float ANGLE_FUSE_ALPHA = 0.995f;

// Outer angle loop (P only -> generates desired rate)
static constexpr float ANGLE_KP_ROLL  = 3.0f;   // deg -> deg/s
static constexpr float ANGLE_KP_PITCH = 3.0f;   // deg -> deg/s
static constexpr float MAX_ANGLE_RATE_DPS = 30.0f;

// Inner rate loop (P only baseline)
static constexpr float RATE_KP_ROLL  = 0.0015f;
static constexpr float RATE_KP_PITCH = 0.0015f;
static constexpr float RATE_KP_YAW   = 0.0020f;

// Gyro low-pass in control loop
static constexpr float GYRO_LPF_ALPHA = 0.6f;

// Mixer output clamps in thrust-fraction units
static constexpr float ROLL_OUT_MAX  = 0.15f;
static constexpr float PITCH_OUT_MAX = 0.15f;
static constexpr float YAW_OUT_MAX   = 0.12f;

// Axis sign helpers for quick debugging
// If angle mode reacts the wrong way, flip one of these first.
static constexpr float ROLL_RATE_SIGN  = +1.0f;
static constexpr float PITCH_RATE_SIGN = +1.0f;
static constexpr float YAW_RATE_SIGN   = +1.0f;

static constexpr float ACC_ROLL_SIGN   = +1.0f;
static constexpr float ACC_PITCH_SIGN  = +1.0f;

// ================= ALTITUDE CONTROL =================
static float altitudeFiltered = 0.0f;

static float altitudeTarget = 1.0f;   // 1 meter hover

static constexpr float ALT_KP = 0.8f;
static constexpr float ALT_KI = 0.4f;
static constexpr float ALT_KD = 0.3f;

static float altIntegral = 0.0f;
static float altPrevError = 0.0f;

static constexpr uint32_t ALT_PERIOD_US = 20000; // 50 Hz
static uint32_t lastAltUs = 0;

static float groundPressure = 101325.0f;
static float pressureFiltered = 101325.0f;

// ===================== Logging =====================
static void logLine(const char* msg) {
  Serial.print("[AIR] ");
  Serial.println(msg);
}

// ===================== Helpers =====================
static inline float clamp_f(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ===================== Motors =====================
static void motorsSetAll(uint16_t us) {
  tLib.motors.set(1, us);
  tLib.motors.set(2, us);
  tLib.motors.set(3, us);
  tLib.motors.set(4, us);
}

static uint16_t m1_cmd_us = 1000, m2_cmd_us = 1000, m3_cmd_us = 1000, m4_cmd_us = 1000;

// ===================== Sensors (acc in g, gyro in dps) =====================
static float ax=0, ay=0, az=0;
static float gx=0, gy=0, gz=0;

static float ax_g=0, ay_g=0, az_g=0;
static float gx_dps=0, gy_dps=0, gz_dps=0;

// accel bias (in g)
static double accelBiasX=0, accelBiasY=0, accelBiasZ=0;

// gyro bias (in dps)
static double gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;

// low-pass for gyro raw during calibration
static float ogx=0, ogy=0, ogz=0;

// exported telemetry angles
static float roll_deg=0, pitch_deg=0, yaw_deg=0;

// complementary filter state
static float rollEst_deg = 0.0f;
static float pitchEst_deg = 0.0f;
static bool angleInitialized = false;

// IMU timing
static uint32_t lastImuUs = 0;
static constexpr uint32_t IMU_PERIOD_US = 5000;

// --- calibration adapted to your library units (accel in g, gyro in dps) ---
static void calibrateAccel() {
  double sumAx=0, sumAy=0, sumAz=0;
  const int samples = 3000;
  for(int i=0; i<samples; i++){
    tLib.imu.readAccel(ax, ay, az); // g
    sumAx += ax; sumAy += ay; sumAz += az;
    delayMicroseconds(500);
  }
  accelBiasX = sumAx / samples;
  accelBiasY = sumAy / samples;
  accelBiasZ = (sumAz / samples) - 1.0; // assumes rest ~ +1g on Z
}

static void calibrateGyro() {
  gyroBiasX = gyroBiasY = gyroBiasZ = 0;
  ogx = ogy = ogz = 0;

  const int samples = 3000;
  for (int i=0;i<samples;i++){
    tLib.imu.readGyro(gx, gy, gz); // dps
    const float alphaA = 0.2f;
    ogx += alphaA * (gx - ogx); gx = ogx;
    ogy += alphaA * (gy - ogy); gy = ogy;
    ogz += alphaA * (gz - ogz); gz = ogz;

    gyroBiasX += gx;
    gyroBiasY += gy;
    gyroBiasZ += gz;
    delayMicroseconds(500);
  }
  gyroBiasX /= samples;
  gyroBiasY /= samples;
  gyroBiasZ /= samples;
}

// ===================== SENSOR STEP =====================
// Reads IMU, updates bias-corrected gyro, and computes complementary-filter angles.
static void sensorStep(uint32_t nowUs){
  if ((int32_t)(nowUs - lastImuUs) < (int32_t)IMU_PERIOD_US) return;

  const uint32_t prevUs = lastImuUs;
  lastImuUs = nowUs;

  float dt = (nowUs - prevUs) * 1e-6f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

  // read sensors
  tLib.imu.readAccel(ax, ay, az); // g
  tLib.imu.readGyro(gx, gy, gz);  // dps

  // accel bias correction
  ax_g = ax - (float)accelBiasX;
  ay_g = ay - (float)accelBiasY;
  az_g = az - (float)accelBiasZ;

  // adaptive gyro bias update when stable
  float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
  if(gyroMag < 0.5f && fabsf(ax_g) < 0.15f && fabsf(ay_g) < 0.15f && fabsf(az_g - 1.0f) < 0.20f){
    const float biasAlpha = 0.03f;
    gyroBiasX += biasAlpha * ((double)gx - gyroBiasX);
    gyroBiasY += biasAlpha * ((double)gy - gyroBiasY);
    gyroBiasZ += biasAlpha * ((double)gz - gyroBiasZ);
  }

  // bias-corrected gyro rates
  gx_dps = gx - (float)gyroBiasX;
  gy_dps = gy - (float)gyroBiasY;
  gz_dps = gz - (float)gyroBiasZ;

  // accel-only roll/pitch estimate
  float accRoll_deg =
      ACC_ROLL_SIGN * atan2f(ay_g, az_g) * 57.2957795f;

  float accPitch_deg =
      ACC_PITCH_SIGN * atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.2957795f;

  // complementary filter
  if (!angleInitialized) {
    rollEst_deg = accRoll_deg;
    pitchEst_deg = accPitch_deg;
    angleInitialized = true;
  } else {
    rollEst_deg  += (ROLL_RATE_SIGN  * gx_dps) * dt;
    pitchEst_deg += (PITCH_RATE_SIGN * gy_dps) * dt;

    rollEst_deg  = ANGLE_FUSE_ALPHA * rollEst_deg  + (1.0f - ANGLE_FUSE_ALPHA) * accRoll_deg;
    pitchEst_deg = ANGLE_FUSE_ALPHA * pitchEst_deg + (1.0f - ANGLE_FUSE_ALPHA) * accPitch_deg;
  }

  // telemetry angles
  roll_deg  = rollEst_deg;
  pitch_deg = pitchEst_deg;
  yaw_deg  += (YAW_RATE_SIGN * gz_dps) * dt;
}

// ===================== Telemetry packet =====================
static int32_t telemNum = 0;
static uint32_t nextTelemMs = 0;

// ===================== Commands =====================
static void handleCommand(const Command &c) {
  // Safely copy and null-terminate
  char buf[sizeof(c.cmd) + 1];
  memcpy(buf, c.cmd, sizeof(c.cmd));
  buf[sizeof(c.cmd)] = '\0';

  // Strip trailing whitespace
  size_t len = strlen(buf);
  while (len > 0 && (buf[len - 1] == ' ' || buf[len - 1] == '\t' ||
                     buf[len - 1] == '\r' || buf[len - 1] == '\n')) {
    buf[--len] = '\0';
  }

  if (strcmp(buf, "arm") == 0) {
    motorsArmed = true;
    Serial.println("[AIR] ARMED");
  } else if (strcmp(buf, "disarm") == 0) {
    motorsArmed = false;
    motorsSetAll(1000);
    Serial.println("[AIR] DISARMED");
  } else if (strcmp(buf, "ping") == 0) {
    Serial.println("PONG");
  } else {
    Serial.print("[AIR] unknown cmd -> ");
    Serial.println(buf);
  }
}

// ===================== Setup / loop =====================
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(921600);

  logLine("Boot");

  int status = tLib.init();
  if (status != 1) {
    Serial.print("[AIR] Tonttulib init failed code: ");
    Serial.println(status);
    while (true) {}
  }

  tLib.led.off();

  // safe motors
  motorsSetAll(1000);

  // Serial.println("[AIR] Calibrating accel...");
  // calibrateAccel();
  Serial.println("[AIR] Calibrating gyro...");
  calibrateGyro();
  Serial.println("[AIR] Calibration done.");

  groundPressure = tLib.baro.readPressure();
  while (!isfinite(groundPressure) || groundPressure < 80000.0f || groundPressure > 110000.0f) {
    delay(10);
    groundPressure = tLib.baro.readPressure();
  }
  pressureFiltered = groundPressure;

  lastImuUs = micros();
  nextTelemMs = millis() + TELEMETRY_PERIOD_MS;
}

void loop() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  tLib.update();
  sensorStep(nowUs);

  // Check for commands from bridge
  Command cmd;
  while (receivePacket(Serial1, cmd)) {
    handleCommand(cmd);
  }

  // ================= Low Voltage Cutoff =================
  float lipoV = tLib.vlipo.readVoltage();
  if (lipoV < 6.0f) {
    motorsArmed = false;
    baseThrust = 0.0f;
    motorsSetAll(1000);
    return;
  }

  // ================= Fixed-rate control =================
  static uint32_t lastCtrlUs = 0;
  if (lastCtrlUs == 0) {
    lastCtrlUs = nowUs;
  }

  if ((int32_t)(nowUs - lastCtrlUs) >= (int32_t)CTRL_PERIOD_US) {
    lastCtrlUs += CTRL_PERIOD_US;

    // ================= Gyro Filtering =================
    static float gx_f = 0.0f;
    static float gy_f = 0.0f;
    static float gz_f = 0.0f;

    gx_f += GYRO_LPF_ALPHA * (gx_dps - gx_f);
    gy_f += GYRO_LPF_ALPHA * (gy_dps - gy_f);
    gz_f += GYRO_LPF_ALPHA * (gz_dps - gz_f);

    // ================= Axis Mapping =================
    const float rollRate_dps  = ROLL_RATE_SIGN  * gx_f;
    const float pitchRate_dps = PITCH_RATE_SIGN * gy_f;
    const float yawRate_dps   = YAW_RATE_SIGN   * gz_f;

    // ================= Outer angle loop (P only) =================
    const float rollAngleSp_deg  = 0.0f;
    const float pitchAngleSp_deg = 0.0f;
    const float yawSp_dps        = 0.0f;

    float rollSp_dps  = ANGLE_KP_ROLL  * (rollAngleSp_deg  - rollEst_deg);
    float pitchSp_dps = ANGLE_KP_PITCH * (pitchAngleSp_deg - pitchEst_deg);

    rollSp_dps  = clamp_f(rollSp_dps,  -MAX_ANGLE_RATE_DPS, MAX_ANGLE_RATE_DPS);
    pitchSp_dps = clamp_f(pitchSp_dps, -MAX_ANGLE_RATE_DPS, MAX_ANGLE_RATE_DPS);

    // ================= Inner rate loop (P only) =================
    float rollOut  = RATE_KP_ROLL  * (rollSp_dps  - rollRate_dps);
    float pitchOut = RATE_KP_PITCH * (pitchSp_dps - pitchRate_dps);
    float yawOut   = RATE_KP_YAW   * (yawSp_dps   - yawRate_dps);

    rollOut  = clamp_f(rollOut,  -ROLL_OUT_MAX,  ROLL_OUT_MAX);
    pitchOut = clamp_f(pitchOut, -PITCH_OUT_MAX, PITCH_OUT_MAX);
    yawOut   = clamp_f(yawOut,   -YAW_OUT_MAX,   YAW_OUT_MAX);

    // ================= PLUS FRAME MIXER =================
    float m1 = baseThrust + rollOut + yawOut;
    float m2 = baseThrust + pitchOut - yawOut;
    float m3 = baseThrust - rollOut + yawOut;
    float m4 = baseThrust - pitchOut - yawOut;

    // ================= PWM Mapping =================
    m1_cmd_us = (uint16_t)constrain((int)(m1 * 1000.0f + 1000.0f), 1000, 2000);
    m2_cmd_us = (uint16_t)constrain((int)(m2 * 1000.0f + 1000.0f), 1000, 2000);
    m3_cmd_us = (uint16_t)constrain((int)(m3 * 1000.0f + 1000.0f), 1000, 2000);
    m4_cmd_us = (uint16_t)constrain((int)(m4 * 1000.0f + 1000.0f), 1000, 2000);

    // ================= Motor Output =================
    if (!motorsArmed) {
      motorsSetAll(1000);
    } else {
      tLib.motors.set(1, m1_cmd_us);
      tLib.motors.set(2, m2_cmd_us);
      tLib.motors.set(3, m3_cmd_us);
      tLib.motors.set(4, m4_cmd_us);
    }
  }

  // ================= ALTITUDE CONTROL + SETPOINT RAMP (50 Hz) =================
  if ((int32_t)(nowUs - lastAltUs) >= (int32_t)ALT_PERIOD_US) {
    lastAltUs += ALT_PERIOD_US;

    // --- Setpoint ramp ---
    if (motorsArmed) {
      altitudeTarget += 0.002f;
      altitudeTarget = clamp_f(altitudeTarget, 0.0f, 1.0f);
    } else {
      altitudeTarget = 0.0f;
    }

    // --- Baro read with NaN/range guard (Pa) ---
    float pressure = tLib.baro.readPressure();
    if (!isfinite(pressure) || pressure < 80000.0f || pressure > 110000.0f) {
      pressure = pressureFiltered;
    }

    pressureFiltered += 0.1f * (pressure - pressureFiltered);

    float altitude =
        44330.0f *
        (1.0f - powf(pressureFiltered / groundPressure, 0.1903f));

    altitudeFiltered = altitude;

    float altError = altitudeTarget - altitude;

    altIntegral += altError * 0.02f;
    altIntegral = clamp_f(altIntegral, -0.3f, 0.3f);

    float altOutput =
        ALT_KP * altError +
        ALT_KI * altIntegral;

    baseThrust = clamp_f(
      0.56f + altOutput,
      THRUST_MIN,
      THRUST_MAX
    );
  }

  // ================= Telemetry =================
  if ((int32_t)(nowMs - nextTelemMs) >= 0) {
    nextTelemMs += TELEMETRY_PERIOD_MS;

    SensorData sd;
    sd.pressure = pressureFiltered;
    sd.num      = telemNum++;
    snprintf(sd.state, sizeof(sd.state), "%s", motorsArmed ? "ARMED" : "DISARMED");
    sd.lat = 0.0f;
    sd.lon = 0.0f;
    sendPacket(Serial1, sd);
  }
}
