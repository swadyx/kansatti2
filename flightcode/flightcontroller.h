#pragma once
#include <Arduino.h>

namespace FC {

void init();
void update(uint32_t nowUs);

void arm();
void disarm();

// KORJAUS: parametrit poistettu vastaamaan toteutusta
void setTarget();

}
