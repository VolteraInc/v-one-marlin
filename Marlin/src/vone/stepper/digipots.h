#pragma once

#include <inttypes.h>

void digiPotInit();
void digiPotSetCurrent(uint8_t axis, uint8_t current);
uint8_t digiPotGetCurrent(uint8_t axis);
