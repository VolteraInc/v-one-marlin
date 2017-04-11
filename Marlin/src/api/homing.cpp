#include "api.h"

#include "../../Marlin.h"
#include "../../planner.h"
#include "../../stepper.h"

static signed char axis_homed_state[3] = {0, 0, 0};
const float homing_feedrate[] = HOMING_FEEDRATE;


#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);


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

  setHomedState(axis, home_dir(axis));

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Homed "); SERIAL_ECHO(axis_codes[axis]); SERIAL_ERRORPGM("-axis");
    SERIAL_ECHOPGM(" base_home_pos:"); SERIAL_ECHO(base_home_pos(axis));
    SERIAL_ECHOPGM(" base_min_pos:"); SERIAL_ECHO(base_min_pos(axis));
    SERIAL_ECHOPGM(" current_position:"); SERIAL_ECHO(current_position[axis]);
    SERIAL_ECHOPGM(" base_min_pos:"); SERIAL_ECHO(base_min_pos(axis));
    SERIAL_ECHOPGM(" base_max_pos:"); SERIAL_ECHO(base_max_pos(axis));
    SERIAL_ECHOPGM("\n");
  }
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
  float measurement;
  if (measureAtSwitch(axis, home_dir(axis), useDefaultMaxTravel, measurement)) {
    goto DONE;
  }

  if (axis == X_AXIS || axis == Y_AXIS) {
    // Move slightly away from switch
    // Note: Otherwise we will not be able to go to 0,0 without
    // hitting a limit switch (and messing up our position)
    if (logging_enabled) {
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Retracting from home switch");
    }
    if (retractFromSwitch(axis, home_dir(axis), HOMING_XY_OFFSET)) {
      goto DONE;
    }
  }

  // Set current position as home
  axisIsAtHome(axis);

  returnValue = 0;

DONE:
  plan_enable_skew_adjustment(true);
  return returnValue;
}

int rawHome(Tool tool, bool homingX, bool homingY, bool homingZ) {
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

int moveToZSwitchXY(Tool tool) {
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
  return moveXY(tool, min_z_x_pos, min_z_y_pos);
}

int homeZ(Tool tool) {

  switch(tool) {
    case TOOLS_DISPENSER:
    case TOOLS_PROBE:
      break;
    default:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to home Z-axis, no tool attached");
      return -1;
  }

  // Home Z to the z-switch
  if (
    moveToZSwitchXY(tool) ||
    s_homeAxis(Z_AXIS)
  ) {
    return -1;
  }

  if (readToolState(tool) == TOOL_STATE_TRIGGERED) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to home Z-axis, probe triggered before contacting z-switch");
    return -1;
  }

  // Raise and set the max-z soft limit
  // Note: the point of contact can vary slightly, so we add some fudge to make to max tolerant
  const float fudge = 0.5; // mm
  if(raise()) {
    return -1;
  }
  max_pos[Z_AXIS] = current_position[Z_AXIS] + fudge;

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("setting soft limit max-z to "); SERIAL_ECHOLN(max_pos[Z_AXIS]);
  }

  return 0;
}

int homeXY() {
  return (
    raise() ||
    rawhome(TOOLS_NONE, true, true, false)
  );
}
