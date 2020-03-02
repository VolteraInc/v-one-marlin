#pragma once

#include "../../../Axis.h"

void zeroAxisAtCurrentPosition(AxisEnum axis, float homingOffset = 0.0f);
int homeHorizontalAxis(const Endstop& endstop, float offset = 0.0f);