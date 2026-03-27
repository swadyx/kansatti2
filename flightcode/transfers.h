#pragma once
#include <stdint.h>

namespace Transfers {

// --- Flight events ---
bool launchDetected(float accX, float accY, float accZ);
bool deploymentDetected(float ldr);
bool stableDetected();
bool landingDetected(float p0, float pCurrent);

}
