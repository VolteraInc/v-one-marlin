#include "api.h"

#include "../../Marlin.h"
#include "../vone/vone.h"
#include "../../planner.h"
#include "../../stepper.h"

// Sets direction of endstops when homing; 1=MAX, -1=MIN
static const signed char home_dir[] = { -1, -1, -1 };

static signed char axis_homed_state[3] = {0, 0, 0};
const float homing_feedrate[] = HOMING_FEEDRATE;


bool homedXY() {
  return getHomedState(X_AXIS) && getHomedState(Y_AXIS);
}

bool homedZ() {
  return getHomedState(Z_AXIS);
}

int getHomedState(int axis) {
  return axis_homed_state[axis];
}

void setHomedState(int axis, int value) {
  if (axis_homed_state[axis] != value) {
    SERIAL_ECHO_START;
    SERIAL_PAIR("Homed ", axis_codes[axis]); SERIAL_ECHOPGM("-axis");
    SERIAL_EOL;

    axis_homed_state[axis] = value;
    sendHomedStatusUpdate();

    // When reseting Z, reset it's soft limit too
    if (axis == Z_AXIS && value == 0) {
      max_pos[Z_AXIS] = Z_MAX_POS;
    }
  }
}

void sendHomedStatusUpdate() {
  SERIAL_PROTOCOLPGM("homedStatusUpdate");
  SERIAL_PROTOCOLPGM(" x:"); SERIAL_PROTOCOL(getHomedState(X_AXIS));
  SERIAL_PROTOCOLPGM(" y:"); SERIAL_PROTOCOL(getHomedState(Y_AXIS));
  SERIAL_PROTOCOLPGM(" z:"); SERIAL_PROTOCOL(getHomedState(Z_AXIS));
  SERIAL_PROTOCOLPGM("\n");
}

static void axisIsAtHome(int axis) {
  current_position[axis] = 0;

  plan_set_position(
    current_position[ X_AXIS ],
    current_position[ Y_AXIS ],
    current_position[ Z_AXIS ],
    current_position[ E_AXIS ]
  );

  setHomedState(axis, home_dir[axis]);
}

static int s_homeAxis(int axis) {
  int returnValue = -1;
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("home axis:"); SERIAL_ECHOLN(axis_codes[axis]);
  }

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
  current_position[axis] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  // Move to the switch
  // Note: we use measureAtSwitch so that we contact the switch accurately
  // TODO: use measureAtSwitchRelease for homing?
  float measurement;
  if (measureAtSwitch(axis, home_dir[axis], useDefaultMaxTravel, measurement)) {
    goto DONE;
  }

  switch(axis) {
    case X_AXIS:
    case Y_AXIS:
      // Move slightly away from switch
      // Note: Otherwise we will not be able to go to 0,0 without
      // hitting a limit switch (and messing up our position)
      if (logging_enabled) {
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Retracting from home switch");
      }
      if (retractFromSwitch(axis, home_dir[axis], HOMING_XY_OFFSET)) {
        goto DONE;
      }

    case Z_AXIS:
      // Confirm probe not triggered
      // NOTE:
      //    1) if the probe triggers before the z-switch, it suggests
      //       that excessive force is needed to trigger the z-switch
      //       which could result in broken nozzle when we home the
      //       dispenser.
      //    2) The Drill's mounted voltages registers as 'triggered'
      //       if we do a digital read on it.
      // DEFER: This should happen in measureAtSwitch, i.e. take a pint
      //        and confirm it triggers (instead of taking an axis)
      if (vone->toolBox.probe.attached()) {
        bool triggered;
        if (vone->pins.ptop.readDigitalValue(triggered)) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("Unable to home Z-axis, could determine the state of other switches");
          goto DONE;
        }

        if (triggered) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("Unable to home Z-axis, another switch triggered before the z-switch");
          goto DONE;
        }
      }
  }

  // Set current position as home
  axisIsAtHome(axis);

  returnValue = 0;

DONE:
  plan_enable_skew_adjustment(true);
  return returnValue;
}

int rawHome(tools::Tool& tool, bool homingX, bool homingY, bool homingZ) {
  // Homing Y first moves the print head out of the way, which
  // which allows the user to access the board/bed sooner
  if (homingY) {
    if (s_homeAxis(Y_AXIS)) {
      return -1;
    }
  }

  if (homingX) {
    if (s_homeAxis(X_AXIS)) {
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
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Move to z-switch's x,y position");
  }

  if (!homedXY()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to move to Z-Switch, either the x-axis or the y-axis has not been homed");
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

int homeZ(tools::Tool& tool) {

  // Home Z to the z-switch
  if (
    moveToZSwitchXY(tool) ||
    s_homeAxis(Z_AXIS)
  ) {
    return -1;
  }

  // Raise and set the max-z soft limit
  // Note: the point of contact can vary slightly, so we add some fudge to make to max tolerant
  const float fudge = 0.5; // mm
  if(raise()) {
    return -1;
  }
  max_pos[Z_AXIS] = current_position[Z_AXIS] + fudge;

  SERIAL_ECHO_START;
  SERIAL_PAIR("setting soft limit max-z to ", max_pos[Z_AXIS]);
  SERIAL_EOL;

  return 0;
}

int homeXY(tools::Tool& tool) {
  return (
    raise() ||
    rawHome(tool, true, true, false)
  );
}
