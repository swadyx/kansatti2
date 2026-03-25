#pragma once
#include <Arduino.h>

namespace Sensors {

void init();
void update(uint32_t nowUs);
void updateVelocity(uint32_t nowUs); // KORJAUS: puuttui

float getRoll();
float getPitch();
float getYaw();

float getGroundPressure();
float getPressure();
float setGroundPressure();

float getGyroX();
float getGyroY();
float getGyroZ();

float getAccX();
float getAccY();
float getAccZ();

bool gpsHasFix();
bool gpsHasNew();

float getLDR();

double getLat();
double getLon();

float velX();
float velY();

}
