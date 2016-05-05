#include "api.h"

#include "../Marlin.h"
#include "../planner.h"
#include "../stepper.h"

signed char axis_homed_state[3] = {0, 0, 0};
float homing_feedrate[] = HOMING_FEEDRATE;


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
  SERIAL_PROTOCOL("homedStatusUpdate");
  SERIAL_PROTOCOL(" x:"); SERIAL_PROTOCOL(getHomedState(X_AXIS));
  SERIAL_PROTOCOL(" y:"); SERIAL_PROTOCOL(getHomedState(Y_AXIS));
  SERIAL_PROTOCOL(" z:"); SERIAL_PROTOCOL(getHomedState(Z_AXIS));
  SERIAL_PROTOCOL("\n");
}

static void axisIsAtHome(int axis) {
  current_position[axis] = base_home_pos(axis);

  min_pos[axis] = base_min_pos(axis);
  max_pos[axis] = base_max_pos(axis);

  plan_set_position(
    current_position[ X_AXIS ],
    current_position[ Y_AXIS ],
    current_position[ Z_AXIS ],
    current_position[ E_AXIS ]
  );

  setHomedState(axis, home_dir(axis));

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHO("Homed "); SERIAL_ECHO(axis_codes[axis]); SERIAL_ERROR("-axis");
    SERIAL_ECHO(" base_home_pos:"); SERIAL_ECHO(base_home_pos(axis));
    SERIAL_ECHO(" base_min_pos:"); SERIAL_ECHO(base_min_pos(axis));
    SERIAL_ECHO(" current_position:"); SERIAL_ECHO(current_position[axis]);
    SERIAL_ECHO(" base_min_pos:"); SERIAL_ECHO(base_min_pos(axis));
    SERIAL_ECHO(" base_max_pos:"); SERIAL_ECHO(base_max_pos(axis));
    SERIAL_ECHO("\n");
  }
}

static int homeAxis(int axis) {
  int returnValue = -1;
  SERIAL_ECHO_START;
  SERIAL_ECHO("home axis:"); SERIAL_ECHO(axis_codes[axis]);
  SERIAL_ECHO("\n");

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // To prevent crashes, raise Z
  if (axis == X_AXIS || axis == Y_AXIS) {
    raise();
  }

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

  axisIsAtHome(axis);

  returnValue = 0;

DONE:
  plan_enable_skew_adjustment(true);
  return returnValue;
}

int home(bool homingX, bool homingY, bool homingZ) {

  // Homing Y first moves the print head out of the way, which
  // which allows the user to access the board/bed sooner
  if (homingY) {
    if (homeAxis(Y_AXIS)) {
      return -1;
    }
  }

  if (homingX) {
    if (homeAxis(X_AXIS)) {
      return -1;
    }
  }

  if (homingZ) {
    if (homeZ()) {
      return -1;
    }
  }

  // Success
  return 0;
}

int moveToZSwitch() {
  if (min_z_x_pos != current_position[X_AXIS] || min_z_y_pos != current_position[Y_AXIS]) {
    if (raise()) {
      return -1;
    }
  }
  return moveXY(min_z_x_pos, min_z_y_pos);
}

int homeZ() {
  // Home Z to the z-switch
  if (
    moveToZSwitch() ||
    homeAxis(Z_AXIS)
  ) {
    return -1;
  }

  // Raise and set the max-z soft limit
  if(raise()) {
    return -1;
  }
  max_pos[Z_AXIS] = current_position[Z_AXIS];

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHO("setting soft limit max-z to "); SERIAL_ECHOLN(max_pos[Z_AXIS]);
  }

  return 0;
}

int homeXY() {
  return home(true, true, false);
}