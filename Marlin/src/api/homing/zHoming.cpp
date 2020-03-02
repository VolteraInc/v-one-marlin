#include "../api.h"

#include "../../../Marlin.h"
#include "../../vone/VOne.h"

#include "internal.h"

int moveToZSwitchXY(tools::Tool& tool) {
  log << F("Move to z-switch's x,y position") << endl;

  if (!homedXY()) {
    logError
      << F("Unable to move to Z-Switch, ")
      << F("either the x-axis or the y-axis has not been homed")
      << endl;
    return -1;
  }

  if (min_z_x_pos != current_position[X_AXIS] || min_z_y_pos != current_position[Y_AXIS]) {
    if (raise(tool)) {
      return -1;
    }
  }

  return moveXY(tool, min_z_x_pos, min_z_y_pos);
}

int homeZ(tools::Tool& tool, float offset) {
  log << F("homing z-axis") << endl;

  // Finish any pending moves (prevents crashes)
  vone->stepper.finishPendingMoves();

  // Clear homed state
  // Note: movement safety checks may fail while homing, clearing
  // the homed state disables those safety checks.
  setHomedState(Z_AXIS, 0);

  // Determine the max-z (soft limit)
  // Note: the point of contact can vary slightly, so we back off a small
  //       amount to avoid triggering the switch during raiseToSoftMax
  const float zMaxOffset = 0.5; // mm

  // Z-homing sequence
  // Note: contacts zMax and zSwitch, so we can determine max_pos later
  float zMaxMeasurement;
  float zSwitchMeasurement;
  if (
    // Make the current location 0, and sync with the planner
    // Note: Doing this means that homing can deal with:
    //   - crazy values for the current_position
    //   - current_position and planner being out of sync
    zeroAxisAtCurrentPosition(Z_AXIS) ||

    // Go to the z-switch
    moveToZSwitchXY(tool) ||

    // Measure the zMax and z-switch locations
    measureAtSwitch(vone->endstops.zMax, useDefaultMaxTravel, zMaxMeasurement) ||
    retractFromSwitch(vone->endstops.zMax, zMaxOffset) ||
    measureAtSwitch(vone->endstops.zSwitch, useDefaultMaxTravel, zSwitchMeasurement) ||

    // Set the home position here
    zeroAxisAtCurrentPosition(Z_AXIS, offset)
  ) {
    logError << F("Unable to home z-axis") << endl;
    return -1;
  }

  // We are home!
  setHomedState(Z_AXIS, -1);

  // Set max-z
  // NOTE: The offset is already included in current_position
  //       when we compute max_pos
  float delta = zMaxMeasurement - zSwitchMeasurement;
  max_pos[Z_AXIS] = current_position[Z_AXIS] + delta;
  log << F("distance between zMax and zSwitch is") << delta << endl;
  log << F("set maximum Z position to ") << max_pos[Z_AXIS] << endl;

  return 0;
}
