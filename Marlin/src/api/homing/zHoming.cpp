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

static const bool skipEstablishSoftMaxZ = false;
static int s_homeZaxis(tools::Tool& tool, float offset, bool shouldEstablishSoftMaxZ = true) {
  const auto& zSwitch = vone->endstops.zSwitch;
  const auto& zMax = vone->endstops.zMax;

  log << F("homing z-axis") << endl;

  // Finish any pending moves (prevents crashes)
  vone->stepper.finishPendingMoves();

  // Clear homed state
  // Note: movement safety checks may fail while homing, clearing
  // the homed state disables those safety checks.
  setHomedState(Z_AXIS, 0);

  // Set up for z-homing
  if (
    // Make the current location 0, and sync with the planner
    // Note: Doing this means that homing can deal with:
    //   - crazy values for the current_position
    //   - current_position and planner being out of sync
    zeroAxisAtCurrentPosition(Z_AXIS) ||

    // Go to the z-switch
    moveToZSwitchXY(tool)
  ) {
    logError << F("Unable to home z-axis, could not move to z-switch") << endl;
    return -1;
  }

  // Measure the zMax switch location, unless skipping
  float zMaxMeasurement;
  if (shouldEstablishSoftMaxZ) {
    if (measureAtSwitch(zMax, useDefaultMaxTravel, zMaxMeasurement)) {
      logError << F("Unable to home z-axis, could not measure maximum height") << endl;
      return -1;
    }
  }

  float zSwitchMeasurement;
  if (
    // Measure the z-switch location
    measureAtSwitch(zSwitch, useDefaultMaxTravel, zSwitchMeasurement) ||

    // Set the home position here
    zeroAxisAtCurrentPosition(Z_AXIS, offset)
  ) {
    logError << F("Unable to home z-axis, could not measure") << zSwitch.name << endl;
    return -1;
  }

  // We are home!
  setHomedState(Z_AXIS, -1);

  if (shouldEstablishSoftMaxZ) {
    // Set max-z
    // NOTE: The offset is already included in current_position
    //       when we compute max_pos
    float delta = zMaxMeasurement - zSwitchMeasurement;
    float triggerAvoidanceBuffer = 0.5;
    log << F("distance between zMax and zSwitch is ") << delta << endl;
    establishSoftMaxZ(current_position[Z_AXIS] + delta - triggerAvoidanceBuffer);
  }

  return 0;
}

int homeZOnly(tools::Tool& tool, float offset) {
  return s_homeZaxis(tool, offset, skipEstablishSoftMaxZ);
}

int homeZandEstablishSoftMax(tools::Tool& tool, float offset) {
  return s_homeZaxis(tool, offset);
}
