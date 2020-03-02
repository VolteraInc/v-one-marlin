#pragma once

#include "../../../Axis.h"

class Endstop;
class Stepper;

int zeroAxisAtCurrentPosition(AxisEnum axis, float homingOffset = 0.0f);
int homeHorizontalAxis(Stepper& stepper, const Endstop& endstop, float offset = 0.0f);
