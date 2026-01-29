#include <Wire.h>
#include "BMI088.h"
#include <avr/pgmspace.h>

// NINO you can change this
unsigned long imuIntervalMs = 20;      

#define GPS_BYTE_TIMEOUT_MS 250

float ax, ay, az;
float gx, gy, gz;
int16_t temp;

constexpr float MG_TO_G = 1.0f / 1000.0f;

BMI088 bmi088(BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS);

// ----- Timing trackers -----
unsigned long lastGpsTime = 0;
unsigned long lastImuTime = 0;

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  // 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  // 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate
  // 0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz) (risky)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz) (risky and slow)
  // 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz) (slow)
};

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned short year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned char second;
  char valid;
  unsigned long tAcc;
  long nano;
  unsigned char fixType;
  char flags;
  unsigned char reserved1;
  unsigned char numSV;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
  long velN;
  long velE;
  long velD;
  long gSpeed;
  long heading;
  unsigned long sAcc;
  unsigned long headingAcc;
  unsigned short pDOP;
  short reserved2;
  unsigned long reserved3;
};

NAV_PVT pvt;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

// black magic from this madlad: https://www.youtube.com/watch?v=ylxwOg2pXrc
// and of course modified by GPT
bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  static uint32_t lastByteTime = 0;

  const int payloadSize = sizeof(NAV_PVT);

  while (Serial2.available()) {
    byte c = Serial2.read();
    uint32_t now = millis();

    if (lastByteTime != 0 && (now - lastByteTime) > GPS_BYTE_TIMEOUT_MS) {
      // Partial packet is invalid — drop it
      fpos = 0;
      Serial.println("too long delay, dropping the current gps packet");
    }
    lastByteTime = now;

    if (fpos < 2) {
      if (c == UBX_HEADER[fpos]) {
        fpos++;
      } else {
        fpos = 0;
      }
    }
    else {
      if ((fpos - 2) < payloadSize) {
        ((unsigned char*)(&pvt))[fpos - 2] = c;
      }

      fpos++;

      if (fpos == (payloadSize + 2)) {
        calcChecksum(checksum);
      }
      else if (fpos == (payloadSize + 3)) {
        if (c != checksum[0]) {
          fpos = 0;
        }
      }
      else if (fpos == (payloadSize + 4)) {
        fpos = 0;
        if (c == checksum[1]) {
          return true;
        }
      }
      else if (fpos > (payloadSize + 4)) {
        fpos = 0;
      }
    }
  }
  return false;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(9600);

  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    Serial2.write(pgm_read_byte(UBLOX_INIT + i));
    delay(5);
  }

  delay(1000);

  while (1) {
    if (bmi088.isConnection()) {
      bmi088.initialize();
      break;
    }
    delay(2000);
  }

  bmi088.setGyroPoweMode(GYRO_NORMAL);
  bmi088.setGyroScaleRange(RANGE_500);
  bmi088.setGyroOutputDataRate(ODR_200_BW_23);

  bmi088.setAccPoweMode(ACC_ACTIVE);
  bmi088.setAccScaleRange(RANGE_6G);
  bmi088.setAccOutputDataRate(ODR_200);
}    

void loop() {
  unsigned long now = millis();

  if (processGPS()) {
    Serial.print("fixType: "); Serial.print(pvt.fixType);
    Serial.print(", lat/lon: ");
    Serial.print(pvt.lat / 10000000.0f, 7);
    Serial.print(",");
    Serial.print(pvt.lon / 10000000.0f, 7);
    Serial.print(", altitude: "); Serial.print(pvt.hMSL / 1000.0f, 3);    // meters with 3 decimals
    Serial.print("m heading: "); Serial.print(pvt.heading / 100000.0f, 5);
    Serial.println();
  }


  if (now - lastImuTime >= imuIntervalMs) {
    lastImuTime = now;

    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    temp = bmi088.getTemperature();

    ax *= MG_TO_G;
    ay *= MG_TO_G;
    az *= MG_TO_G;

    Serial.print("ACC[g]: ");
    Serial.print(ax, 4); Serial.print(", ");
    Serial.print(ay, 4); Serial.print(", ");
    Serial.println(az, 4);

    Serial.print("GYRO[deg/s]: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);

    Serial.println("----");
  }
}

