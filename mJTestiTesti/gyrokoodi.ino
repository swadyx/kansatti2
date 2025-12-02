#include "ICM20600.h"
#include <Wire.h>
#include "QuaternionMath.h"
#include "Arduino.h"

ICM20600 icm20600(true);

Vector3 gyroBias = { 0, 0, 0 };

bool initIMU() {
  Wire.begin();
  icm20600.initialize();

  // Set power mode: 6-axis low noise (acc + gyro)
  icm20600.setPowerMode(ICM_6AXIS_LOW_NOISE);

  // Accelerometer ±16g
  icm20600.setAccScaleRange(RANGE_16G);
  icm20600.setAccOutputDataRate(ACC_RATE_1K_BW_99);  // ~100 Hz
  icm20600.setAccAverageSample(ACC_AVERAGE_8);       // averaging filter

  // Gyroscope ±2000 dps
  icm20600.setGyroScaleRange(RANGE_2K_DPS);
  icm20600.setGyroOutputDataRate(GYRO_RATE_1K_BW_41);  // 41 Hz
  icm20600.setGyroAverageSample(GYRO_AVERAGE_4);       // averaging

  // Sample rate divider
  icm20600.setSampleRateDivier(0);


  Serial2.println("Calibrating IMU...");
  // --- Calibration ---
  const int N = 500;  // samples (~0.5s if 1 kHz)
  Vector3 gSum = { 0, 0, 0 };

  for (int i = 0; i < N; i++) {
    // Blink LED fast during calibration
    digitalWrite(PB0, (i % 20 < 10) ? HIGH : LOW);

    gSum.x += icm20600.getGyroscopeX();
    gSum.y += icm20600.getGyroscopeY();
    gSum.z += icm20600.getGyroscopeZ();

    delay(2);  // ~500 Hz effective
  }

  gyroBias.x = (gSum.x / N) * (PI / 180.0f);  // dps → rad/s
  gyroBias.y = (gSum.y / N) * (PI / 180.0f);
  gyroBias.z = (gSum.z / N) * (PI / 180.0f);

  // Indicate calibration done → LED solid ON
  Serial2.println("Calibration done!");
  return true;
}

Vector3 getGyroData() {
  Vector3 gyro;
  gyro.x = icm20600.getGyroscopeX() * (PI / 180.0f) - gyroBias.x;
  gyro.y = icm20600.getGyroscopeY() * (PI / 180.0f) - gyroBias.y;
  gyro.z = -(icm20600.getGyroscopeZ() * (PI / 180.0f) - gyroBias.z);
  return gyro;
}

Vector3 getAccData() {
  Vector3 accel;
  accel.x = (icm20600.getAccelerationX() / 1000.0f);
  accel.y = (icm20600.getAccelerationY() / 1000.0f);
  accel.z = -(icm20600.getAccelerationZ() / 1000.0f);
  return accel;
}

