#include "../api.h"

#include "homing.h"
#include "internal.h"

#include "../../../planner.h"
#include "../../../stepper.h"
#include "../../../serial.h"
#include "../../vone/endstops/Endstop.h"

int homeHorizontalAxis(Stepper& stepper, const Endstop& endstop, float offset) {
  int returnValue = -1;
  const auto axis = endstop.axis;

  log << F("homing axis: ") << axis_codes[axis] << endl;

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Clear homed state
  // Note: movement safety checks may fail while homing, clearing
  // the homed state disables those safety checks.
  setHomedState(axis, 0);

  // Raise flag to let planner know we are homing an axis so it ignores skew adjustments.
  plan_enable_skew_adjustment(false);

  float measurement;
  if (
    // Make the current location 0, and sync with the planner
    // Note: Doing this means that homing can deal with:
    //   - crazy values for the current_position
    //   - current_position and planner being out of sync
    zeroAxisAtCurrentPosition(axis) ||

    // Move to the switch
    // Note: we use measureAtSwitch so that we contact the switch accurately
    measureAtSwitch(endstop, useDefaultMaxTravel, measurement) ||

    // For X and Y we move away from switch slightly
    // Note: Otherwise we will not be able to go to 0,0 without
    //       hitting a limit switch (and messing up our position)
    retractFromSwitch(endstop, HOMING_XY_OFFSET) ||

    // Set the home position here
    zeroAxisAtCurrentPosition(axis, offset)
  ) {
    logError << F("Unable to home ") << axis_codes[axis] << F("-axis") << endl;
    goto DONE;
  }

  // Success
  setHomedState(axis, -1);
  returnValue = 0;

DONE:
  plan_enable_skew_adjustment(true);
  return returnValue;
}
