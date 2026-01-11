#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include "CanSatNeXT.h"

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_BMP280 bmp;
float temperature, pressure, altitude;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[2];

int j1x_val = 2048;
int j1y_val = 2048;
int j2x_val = 2048;
int j2y_val = 2048;

#define CENTER 2048
#define DEADZONE 400

String dir(int v, const char* neg, const char* pos) {
  int diff = v - CENTER;
  if (abs(diff) <= DEADZONE) return "CENTER";
  if (diff > 0) return pos;
  return neg;
}

void kalman_1d(float state, float uncertainty, float input, float measurement) {
  state = state + 0.004 * input;
  uncertainty = uncertainty + 0.004 * 0.004 * 16;
  float gain = uncertainty / (uncertainty + 9);
  state = state + gain * (measurement - state);
  uncertainty = (1 - gain) * uncertainty;
  Kalman1DOutput[0] = state;
  Kalman1DOutput[1] = uncertainty;
}

void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = GyroX / 65.5;
  RatePitch = GyroY / 65.5;
  RateYaw = GyroZ / 65.5;

  AccX = AccXLSB / 4096.0;
  AccY = AccYLSB / 4096.0;
  AccZ = AccZLSB / 4096.0;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.2958;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.2958;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  CanSatInit(28);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 ei vastaa");
    while (1);
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  LoopTimer = micros();
}

void onDataReceived(String data) {
  Serial.println(data);

  int values[4] = {2048,2048,2048,2048};
  int index = 0;
  int start = 0;
  for (int i = 0; i <= data.length(); i++) {
    if (data[i] == ',' || i == data.length()) {
      String part = data.substring(start,i);
      values[index++] = part.toInt();
      start = i + 1;
      if (index >= 4) break;
    }
  }

  j1y_val = values[0];
  j1x_val = values[1];
  j2y_val = values[2];
  j2x_val = values[3];
}

void loop() {
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0;
  altitude = bmp.readAltitude(1013.25);

  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.println("CanSat Telemetry");
  display.print("Temp: "); display.print(temperature); display.println(" C");
  display.print("Pres: "); display.print(pressure); display.println(" hPa");
  display.print("Alt:  "); display.print(altitude); display.println(" m");
  display.print("Roll: "); display.println(KalmanAngleRoll);
  display.print("Pitch:"); display.println(KalmanAnglePitch);

  display.print("T:"); display.print(dir(j1y_val,"DOWN","UP"));
  display.print("  Y:"); display.println(dir(j1x_val,"RIGHT","LEFT"));
  display.print("P:"); display.print(dir(j2y_val,"BACK","FWD"));
  display.print("  R:"); display.println(dir(j2x_val,"RIGHT","LEFT"));

  display.display();

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
