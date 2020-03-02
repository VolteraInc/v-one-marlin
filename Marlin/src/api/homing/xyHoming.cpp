#include "../api.h"

#include "homing.h"
#include "internal.h"

#include "../../../Marlin.h"
#include "../../../planner.h"
#include "../../../stepper.h"
#include "../../vone/endstops/Endstop.h"

int homeHorizontalAxis(const Endstop& endstop, float offset = 0.0f) {
  int returnValue = -1;
  const auto axis = endstop.axis;
  const auto isXorYaxis = axis == X_AXIS || axis == Y_AXIS;

  log << F("zero axis:") << axis_codes[axis] << endl;

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Clear homed state
  // Note: movement safety checks may fail while homing, clearing
  // the homed state disables those safety checks.
  setHomedState(axis, 0);

  // Raise flag to let planner know we are homing an axis so it ignores skew adjustments.
  plan_enable_skew_adjustment(false);

  // Make the current location 0, and sync with the planner
  // Note: Doing this means that homing can deal with:
  //   - crazy values for the current_position
  //   - current_position and planner being out of sync
  zeroAxisAtCurrentPosition(axis);

  // Move to the switch
  // Note: we use measureAtSwitch so that we contact the switch accurately
  float measurement;
  if (measureAtSwitch(endstop, useDefaultMaxTravel, measurement)) {
    goto DONE;
  }

  // For X and Y we move away from switch slightly
  // Note: Otherwise we will not be able to go to 0,0 without
  //       hitting a limit switch (and messing up our position)
  log << F("Retracting from ") << endstop.name << endl;
  if (retractFromSwitch(endstop, HOMING_XY_OFFSET)) {
    goto DONE;
  }

  // We are home!
  zeroAxisAtCurrentPosition(axis, offset);
  setHomedState(axis, -1);

  // Success
  returnValue = 0;

DONE:
  plan_enable_skew_adjustment(true);
  return returnValue;
}
