#include "../../../Axis.h"
#include <math.h>

float axis_steps_per_unit[4];

float stepsToMillimeters(long step, AxisEnum axis) {
  return step / axis_steps_per_unit[axis];
}

long millimetersToSteps(float mm, AxisEnum axis) {
  return lround(mm * axis_steps_per_unit[ axis ]);
}