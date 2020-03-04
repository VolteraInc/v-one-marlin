#include "homing.h"
#include "internal.h"

#include "../movement/movement.h"

#include "../../../Marlin.h"
#include "../../vone/VOne.h"

static int8_t axis_homed_state[3] = {0, 0, 0};
const float homing_feedrate[] = HOMING_FEEDRATE;

bool homedXY() {
  return getHomedState(X_AXIS) && getHomedState(Y_AXIS);
}

int getHomedState(AxisEnum axis) {
  return axis_homed_state[axis];
}

void setHomedState(AxisEnum axis, int value) {
  if (axis_homed_state[axis] != value) {
    log << F("set ") << axis_codes[axis] << F("-axis homed state to ") << value << endl;

    axis_homed_state[axis] = value;
    sendHomedStatusUpdate();

    // When reseting Z, reset it's soft limit too
    if (axis == Z_AXIS && value == 0) {
      clearSoftMaxZ();
    }
  }
}

void sendHomedStatusUpdate() {
  protocol
    << F("homedStatusUpdate")
    << F(" x:") << getHomedState(X_AXIS)
    << F(" y:") << getHomedState(Y_AXIS)
    << F(" z:") << getHomedState(Z_AXIS)
    << endl;
}

int zeroAxisAtCurrentPosition(AxisEnum axis, float homingOffset) {
  // NOTE: Why do we negate the offset?
  //       To offset z-home by 5.0, means we want z=0 to be 5mm higher than
  //       the trigger position of the z-switch. Right now we are at the
  //       trigger postion. We want to tell the planner that 0 is 5mm higher
  //       than the current position. This is the same as telling it we are
  //       currently at -5. That's why there is a negatation here.
  return vone->stepper.overrideCurrentPosition(axis, -homingOffset);
}

int homeXY(tools::Tool& tool) {
  auto& stepper = vone->stepper;
  auto& xMin = vone->endstops.xMin;
  auto& yMin = vone->endstops.yMin;

  return (
    raise(tool) ||
    homeHorizontalAxis(stepper, yMin) ||
    homeHorizontalAxis(stepper, xMin)
  );
}

int rawHomeX() {
  auto& stepper = vone->stepper;
  auto& xMin = vone->endstops.xMin;
  return homeHorizontalAxis(stepper, xMin);
}

int rawHomeY() {
  auto& stepper = vone->stepper;
  auto& yMin = vone->endstops.yMin;
  return homeHorizontalAxis(stepper, yMin);
}
