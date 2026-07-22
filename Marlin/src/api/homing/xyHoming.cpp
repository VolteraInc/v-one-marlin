#include "../api.h"

#include "homing.h"
#include "internal.h"

#include "../../../Marlin.h"
#include "../../../serial.h"
#include "../../vone/endstops/Endstop.h"
#include "../../vone/stepper/Stepper.h"

static int s_measureAtSwitchForHoming(const Endstop& endstop, float& measurement) {
  const auto axis = endstop.axis;

  // A prior failed home can leave the switch triggered. Release it first so
  // the fast measurement below is a real approach rather than a no-op.
  if (endstop.readTriggered() && retractFromSwitch(endstop)) {
    return -1;
  }

  float fastMeasurement;
  if (measureAtSwitch(
    endstop,
    useDefaultMaxTravel,
    fastMeasurement,
    false,
    homing_feedrate[axis]
  )) {
    return -1;
  }

  const float fastTriggerPosition = current_position[axis];
  if (retractFromSwitch(endstop)) {
    return -1;
  }
  const float retractDistance = abs(current_position[axis] - fastTriggerPosition);

  const float slowFeedrate = homing_feedrate[axis] / 6;
  float slowMeasurement;
  if (measureAtSwitch(
    endstop,
    2 * retractDistance,
    slowMeasurement,
    true,
    slowFeedrate
  )) {
    return -1;
  }

  log
    << F("Homing fast/slow switch comparison: ") << endstop.name
    << F(", fast=") << fastMeasurement
    << F(", slow=") << slowMeasurement
    << F(", delta=") << abs(slowMeasurement - fastMeasurement)
    << endl;

  measurement = slowMeasurement;
  return 0;
}

int homeHorizontalAxis(Stepper& stepper, const Endstop& endstop, float offset) {
  int returnValue = -1;
  const auto axis = endstop.axis;

  log << F("homing axis: ") << axis_codes[axis] << endl;

  // Finish any pending moves (prevents crashes)
  stepper.finishPendingMoves();

  // Clear homed state
  // Note: movement safety checks may fail while homing, clearing
  // the homed state disables those safety checks.
  setHomedState(axis, 0);

  // Raise flag to let planner know we are homing an axis so it ignores skew adjustments.
  stepper.enableSkewAdjustment(false);

  float measurement;
  if (
    // Make the current location 0, and sync with the planner
    // Note: Doing this means that homing can deal with:
    //   - crazy values for the current_position
    //   - current_position and planner being out of sync
    zeroAxisAtCurrentPosition(axis) ||

    // Move to the switch
    // Note: we use measureAtSwitch so that we contact the switch accurately
    s_measureAtSwitchForHoming(endstop, measurement) ||

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
  stepper.enableSkewAdjustment();
  return returnValue;
}
