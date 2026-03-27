#pragma once
#include <Arduino.h>

namespace Sensors {

void init(bool recoveredInFlight = false);
void update(uint32_t nowUs, uint8_t state);
void updateVelocity(uint32_t nowUs);

float getRoll();
float getPitch();
float getYaw();

float getGroundPressure();
float getPressure();
float setGroundPressure();
void setGroundPressureTo(float p0);

float getGyroX();
float getGyroY();
float getGyroZ();

float getAccX();
float getAccY();
float getAccZ();

bool gpsHasFix();
bool gpsHasNew();
void gpsConsumeNew();

float getLDR();
float getTemp();

double getLat();
double getLon();

float velX();
float velY();

bool pressureValid();

}
