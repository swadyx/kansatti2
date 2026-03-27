#pragma once
#include <Arduino.h>
#include <Tonttulib.h>

extern Tonttulib tLib;

namespace FC {

extern bool hasTarget;
void setRecovered(bool recovered);

void init();
void update(uint32_t nowUs);

void arm();
void disarm();
bool isArmed();

void beginPoweredFlight(uint32_t nowUs);
bool thrustEnabled(uint32_t nowUs);

void setTarget();
void setStoredTarget(float lat, float lon);

float getTargetLat();
float getTargetLon();

}
