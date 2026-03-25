#pragma once
#include <Arduino.h>

namespace Motor {

void init();
void setAll(uint16_t us);

void set(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

}
