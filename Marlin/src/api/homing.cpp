#include "api.h"

#include "../../Marlin.h"
#include "../vone/VOne.h"
#include "../vone/endstops/Endstop.h"
#include "../../planner.h"
#include "../../stepper.h"

static int8_t axis_homed_state[3] = {0, 0, 0};
const float homing_feedrate[] = HOMING_FEEDRATE;


bool homedXY() {
  return getHomedState(X_AXIS) && getHomedState(Y_AXIS);
}

bool homedZ() {
  return getHomedState(Z_AXIS);
}

int getHomedState(AxisEnum axis) {
  return axis_homed_state[axis];
}

void setHomedState(AxisEnum axis, int value) {
  if (axis_homed_state[axis] != value) {
    log << F("Homed ") << axis_codes[axis] << F("-axis") << endl;

    axis_homed_state[axis] = value;
    sendHomedStatusUpdate();

    // When reseting Z, reset it's soft limit too
    if (axis == Z_AXIS && value == 0) {
      max_pos[Z_AXIS] = Z_MAX_POS;
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

static void s_zeroAxisAtCurrentPosition(AxisEnum axis, float homingOffset = 0.0f) {
  // NOTE: Why do we negate the offset?
  //       To offset z-home by 5.0, means we want z=0 to be 5mm higher than
  //       the trigger position of the z-switch. Right now we are at the
  //       trigger postion. We want to tell the planner that 0 is 5mm higher
  //       than the current position. This is the same as telling it we are
  //       currently at -5. That's why there is a negatation here.
  current_position[axis] = -homingOffset;
  plan_set_position(
    current_position[ X_AXIS ],
    current_position[ Y_AXIS ],
    current_position[ Z_AXIS ],
    current_position[ E_AXIS ]
  );
}

static int s_homeAxis(const Endstop& endstop, float offset = 0.0f) {
  int returnValue = -1;
  const auto axis = endstop.axis;
  const auto isXorYaxis = axis == X_AXIS || axis == Y_AXIS;

#ifdef USE_TRINAMIC_STALL_DETECTION_FOR_HOMING
  int acc_x = max_acceleration_units_per_sq_second[ X_AXIS ];
  int acc_y = max_acceleration_units_per_sq_second[ Y_AXIS ];
#endif

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
  s_zeroAxisAtCurrentPosition(axis);

  // Trinamic Drivers need a minimum speed to detect a stall
  // When we home in X and Y, we don't need very precise measurements, +/- 0.25 mm is probably ok.
  // The XY Positioner will give us the accuracy we require.
  #ifdef USE_TRINAMIC_STALL_DETECTION_FOR_HOMING
  if (isXorYaxis) {
    max_acceleration_units_per_sq_second[ X_AXIS ] = 700;
    max_acceleration_units_per_sq_second[ Y_AXIS ] = 700;
    reset_acceleration_rates();

    // Move in +X and +Y a bit to create space. So we can reach sensorless homing speeds.
    if (retractFromSwitch(endstop, HOMING_XY_OFFSET * 3)) {
      logError
        << F("Unable to create space in  ")
        << axis_codes[axis]
        << F(" axis during homing.")
        << endl;
      goto DONE;
    }
  }
  #endif

  // Move to the switch
  // Note: we use measureAtSwitch so that we contact the switch accurately
  // TODO: use measureAtSwitchRelease for homing?
  float measurement;
  if (measureAtSwitch(endstop, useDefaultMaxTravel, measurement)) {
    goto DONE;
  }

  // For X and Y we move away from switch slightly
  // Note: Otherwise we will not be able to go to 0,0 without
  //       hitting a limit switch (and messing up our position)
  if (isXorYaxis) {
    log << F("Retracting from ") << endstop.name << endl;
    if (retractFromSwitch(endstop, HOMING_XY_OFFSET)) {
      goto DONE;
    }
  }

  // We are home!
  s_zeroAxisAtCurrentPosition(axis, offset);
  setHomedState(axis, -1);

  // Success
  returnValue = 0;

DONE:
  #ifdef USE_TRINAMIC_STALL_DETECTION_FOR_HOMING
  if (isXorYaxis) {
    max_acceleration_units_per_sq_second[ X_AXIS ] = acc_x;
    max_acceleration_units_per_sq_second[ Y_AXIS ] = acc_y;
    reset_acceleration_rates();
  }
  #endif

  plan_enable_skew_adjustment(true);
  return returnValue;
}

int rawHome(tools::Tool& tool, bool homingX, bool homingY, bool homingZ) {
  // Homing Y first moves the print head out of the way, which
  // which allows the user to access the board/bed sooner
  if (homingY) {
    if (s_homeAxis(vone->endstops.yMin)) {
      return -1;
    }
  }

  if (homingX) {
    if (s_homeAxis(vone->endstops.xMin)) {
      return -1;
    }
  }

  if (homingZ) {
    if (homeZ(tool)) {
      return -1;
    }
  }

  // Success
  return 0;
}

// TODO: move to some other file
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
    if (raise()) {
      return -1;
    }
  }

  // TODO: confirm z-switch is not already triggered
  // if z-max is not triggered and z-min is, then // maybe make raise unconditional (add note to prevent reverting)
  //   raise()
  // if z-min is triggered
  //   error

  return moveXY(tool, min_z_x_pos, min_z_y_pos);
}

int homeZ(tools::Tool& tool, float offset) {
  if (
    moveToZSwitchXY(tool) ||
    s_homeAxis(vone->endstops.zSwitch, offset)
  ) {
    return -1;
  }

  // Determine the max-z soft limit
  // Note: the point of contact can vary slightly, so we add some fudge to make to max tolerant
  const float fudge = 0.5; // mm
  if (raise()) {
    return -1;
  }
  max_pos[Z_AXIS] = current_position[Z_AXIS] + fudge;
  log << F("setting soft limit max-z to ") << max_pos[Z_AXIS] << endl;

  return 0;
}

int homeXY(tools::Tool& tool) {
  return (
    raise() ||
    rawHome(tool, true, true, false)
  );
}
