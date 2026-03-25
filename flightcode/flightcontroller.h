#pragma once
#include <Arduino.h>
#include <Tonttulib.h>

extern Tonttulib tLib; 

namespace FC {

void init();
void update(uint32_t nowUs);

void arm();
void disarm();

// KORJAUS: parametrit poistettu vastaamaan toteutusta
void setTarget();

}
