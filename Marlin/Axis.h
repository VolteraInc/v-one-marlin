#pragma once

#include "Configuration.h" // NUM_AXIS

enum AxisEnum {
  X_AXIS = 0,
  Y_AXIS = 1,
  Z_AXIS = 2,
  E_AXIS = 3
};

extern const char axis_codes[NUM_AXIS];
