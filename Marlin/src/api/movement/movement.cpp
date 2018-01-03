#include "../api.h"

#include "../../../Marlin.h"
#include "../../../planner.h"
#include "../../../stepper.h"
#include "../../vone/Vone.h"

static const float s_defaultRetractDistance[] = {
  X_HOME_RETRACT_MM,
  Y_HOME_RETRACT_MM,
  Z_HOME_RETRACT_MM
};


int setPositionEOnly(float e) {
  // Wait for moves to finish before altering the axis
  st_synchronize();

  current_position[E_AXIS] = e;
  plan_set_e_position(e);
  return 0;
}

int setPosition(float x, float y, float z, float e) {
  // Wait for moves to finish before altering the axis
  st_synchronize();

  current_position[X_AXIS] = x;
  current_position[Y_AXIS] = y;
  current_position[Z_AXIS] = z;
  current_position[E_AXIS] = e;
  plan_set_position(
    current_position[X_AXIS],
    current_position[Y_AXIS],
    current_position[Z_AXIS],
    current_position[E_AXIS]
  );
  return 0;
}

// Set the planner position based on the stepper's position.
// Note: Certain movements, like attempting to move past an end-stop, will leave the
// planner out of sync with the stepper. This function corrects the planner's position.
static void s_fixPosition(int axis) {
  current_position[axis] = st_get_position_mm(axis);
  plan_set_position(
    current_position[X_AXIS],
    current_position[Y_AXIS],
    current_position[Z_AXIS],
    current_position[E_AXIS]
  );
  clear_endstop(axis);
}

static float s_maxTravelInAxis(int axis, int direction) {
  switch(axis) {
    case X_AXIS: return getHomedState(X_AXIS) ? X_MAX_LENGTH : X_MAX_LENGTH_BEFORE_HOMING;
    case Y_AXIS: return getHomedState(Y_AXIS) ? Y_MAX_LENGTH : Y_MAX_LENGTH_BEFORE_HOMING;
    case Z_AXIS: return getHomedState(Z_AXIS) ? Z_MAX_LENGTH : (direction < 0 ? Z_MAX_TRAVEL_DOWN_BEFORE_HOMING : Z_MAX_TRAVEL_UP_BEFORE_HOMING);
    default:
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to determine max travel distance for axis, axis "); SERIAL_ERROR(axis);
      SERIAL_ERRORLNPGM(" is not recognized - defaulting to a value of 0");
      return 0; // Will likely result in an obvious error, which we can fix
  }
}

int outputMovementStatus() {
  // Position
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Position");
  SERIAL_ECHOPGM(" X:"); SERIAL_ECHO(current_position[X_AXIS]);
  SERIAL_ECHOPGM(" Y:"); SERIAL_ECHO(current_position[Y_AXIS]);
  SERIAL_ECHOPGM(" Z:"); SERIAL_ECHO(current_position[Z_AXIS]);
  SERIAL_ECHOPGM(" E:"); SERIAL_ECHO(current_position[E_AXIS]);
  SERIAL_ECHOPGM("\n");

  // Stepper status
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("StepperPosition");
  SERIAL_ECHOPGM(" x:"); SERIAL_ECHO(st_get_position_mm(X_AXIS));
  SERIAL_ECHOPGM(" y:"); SERIAL_ECHO(st_get_position_mm(Y_AXIS));
  SERIAL_ECHOPGM(" z:"); SERIAL_ECHO(st_get_position_mm(Z_AXIS));
  SERIAL_ECHOPGM(" e:"); SERIAL_ECHO(st_get_position_mm(E_AXIS));
  SERIAL_ECHOPGM("\n");
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("StepperCounts");
  SERIAL_ECHOPGM(" x:"); SERIAL_ECHO(st_get_position(X_AXIS));
  SERIAL_ECHOPGM(" y:"); SERIAL_ECHO(st_get_position(Y_AXIS));
  SERIAL_ECHOPGM(" z:"); SERIAL_ECHO(st_get_position(Z_AXIS));
  SERIAL_ECHOPGM(" e:"); SERIAL_ECHO(st_get_position(E_AXIS));
  SERIAL_ECHOPGM("\n");

  // Planner
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Planner");
  SERIAL_ECHOPGM(" movesPlanned:"); SERIAL_ECHO((int)movesplanned());
  SERIAL_ECHOPGM("\n");

  // Homing state on each axis
  // 0 = not homed, -1 = homed to min extent, 1 = homed to max extent
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Homing");
  SERIAL_ECHOPGM(" x:"); SERIAL_ECHO(getHomedState(X_AXIS));
  SERIAL_ECHOPGM(" y:"); SERIAL_ECHO(getHomedState(Y_AXIS));
  SERIAL_ECHOPGM(" z:"); SERIAL_ECHO(getHomedState(Z_AXIS));
  SERIAL_ECHOPGM("\n");

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Axis ranges");
  SERIAL_ECHOPGM(" x:"); SERIAL_ECHO(min_pos[X_AXIS]); SERIAL_ECHOPGM(" to "); SERIAL_ECHO(max_pos[X_AXIS]);
  SERIAL_ECHOPGM(" y:"); SERIAL_ECHO(min_pos[Y_AXIS]); SERIAL_ECHOPGM(" to "); SERIAL_ECHO(max_pos[Y_AXIS]);
  SERIAL_ECHOPGM(" z:"); SERIAL_ECHO(min_pos[Z_AXIS]); SERIAL_ECHOPGM(" to "); SERIAL_ECHO(max_pos[Z_AXIS]);
  SERIAL_ECHOPGM("\n");

  return 0;
}

float getDefaultFeedrate() {
  // Returns the max feedrate across all axes
  return 60 * max(
    max(
      max_feedrate[X_AXIS],
      max_feedrate[Y_AXIS]
    ),
    max_feedrate[Z_AXIS]
  );
}

static bool s_moveIsUnsafeInAxis(int axis, float value) {
  // Treat moves prior to homing as safe.
  // Note: We could perform some checks, but there's little value to the user
  if (!getHomedState(axis)) {
    return false;
  }

  // Treat no movement as safe
  // Note: This can also trip if the fudge factor on the z-max is too small
  if (current_position[axis] == value) {
    return false;
  }

  // Out-of-bounds movements are unsafe
  if (value < min_pos[axis] || value > max_pos[axis]) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to move to "); SERIAL_ERROR(value);
    SERIAL_ERRORPGM(" in "); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERRORPGM("-axis, position falls outside of safe bounds\n");
    return true;
  }

  // Movement is safe
  return false;
}

static bool s_moveIsUnsafe(float x, float y, float z) {
  return (
    s_moveIsUnsafeInAxis( X_AXIS, x ) ||
    s_moveIsUnsafeInAxis( Y_AXIS, y ) ||
    s_moveIsUnsafeInAxis( Z_AXIS, z )
  );
}

int asyncRawMove(float x, float y, float z, float e, float f, bool confirmMoveIsSafe) {
  if (confirmMoveIsSafe && s_moveIsUnsafe(x, y, z)) {
    return -1;
  }

  const float speed_in_mm_per_min = f < 0 ? getDefaultFeedrate() : f;

  current_position[ X_AXIS ] = x;
  current_position[ Y_AXIS ] = y;
  current_position[ Z_AXIS ] = z;
  current_position[ E_AXIS ] = e;

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Move");
    SERIAL_ECHOPAIR(" X:", current_position[ X_AXIS ]);
    SERIAL_ECHOPAIR(" Y:", current_position[ Y_AXIS ]);
    SERIAL_ECHOPAIR(" Z:", current_position[ Z_AXIS ]);
    SERIAL_ECHOPAIR(" E:", current_position[ E_AXIS ]);
    SERIAL_ECHOPAIR(" F:", speed_in_mm_per_min);
    SERIAL_EOL;
  }

  // HACK: should not have to pull in stepper link this
  // take Stepper as arg.
  return vone->stepper.add(
    current_position[ X_AXIS ],
    current_position[ Y_AXIS ],
    current_position[ Z_AXIS ],
    current_position[ E_AXIS ],
    speed_in_mm_per_min
  );
}

int relativeRawMoveE(float e, float speed_in_mm_per_min) {

  if(asyncRawMove(
    current_position[ X_AXIS ],
    current_position[ Y_AXIS ],
    current_position[ Z_AXIS ],
    current_position[ E_AXIS ] + e,
    speed_in_mm_per_min,
    skipMovementSafetyCheck
  )) {
    return -1;
  }

  st_synchronize();

  return 0;
}

int relativeRawMoveXYZ(float x, float y, float z, float speed_in_mm_per_min, bool confirmMoveIsSafe) {
  if (asyncRawMove(
    current_position[ X_AXIS ] + x,
    current_position[ Y_AXIS ] + y,
    current_position[ Z_AXIS ] + z,
    current_position[ E_AXIS ],
    speed_in_mm_per_min,
    confirmMoveIsSafe
  )) {
    return -1;
  }

  st_synchronize();

  return 0;
}

int moveToLimit(int axis, int direction, float f, float maxTravel) {
  if(logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Move to limit: "); SERIAL_ECHO(direction < 0 ? '-' : '+'); SERIAL_ECHOLN(axis_codes[axis]);
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Move
  const auto clampedMaxTravel = min(maxTravel, s_maxTravelInAxis(axis, direction));
  const auto travel = direction < 0 ? -clampedMaxTravel : clampedMaxTravel;
  const float clampedSpeed = f < 0 ? homing_feedrate[axis] : min(f, homing_feedrate[axis]);
  if (relativeRawMoveXYZ(
      axis == X_AXIS ? travel : 0.0f,
      axis == Y_AXIS ? travel : 0.0f,
      axis == Z_AXIS ? travel : 0.0f,
      clampedSpeed,
      skipMovementSafetyCheck
    )) {
    return -1;
  }

  // Confirm we triggered
  if (!endstop_triggered(axis)) { // TODO: weak test, should check specific switches
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to move to "); SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERRORPGM(" limit, limit switch did not trigger\n");
    return -1;
  }

  // Resync with true position
  s_fixPosition(axis);
  return 0;
}

int raise() {
  return moveToLimit(Z_AXIS, 1);
}

int retractFromSwitch(int axis, int direction, float retractDistance) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Retract from switch: "); SERIAL_ECHO(direction < 0 ? '-' : '+'); SERIAL_ECHOLN(axis_codes[axis]);
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Retract slightly
  const float distance = retractDistance < 0 ? s_defaultRetractDistance[axis] : retractDistance;
  if(logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Retract by: "); SERIAL_ECHOLN(distance);
  }
  if (relativeRawMoveXYZ(
      axis == X_AXIS ? distance * -direction : 0,
      axis == Y_AXIS ? distance * -direction : 0,
      axis == Z_AXIS ? distance * -direction : 0
    )) {
    return -1;
  }

  // Confirm that the switch was released
  // READ_PIN ?
  if (endstop_triggered(axis)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to retract from "); SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERRORPGM(" switch, switch did not release during retract movement\n");
    return -1;
  }

  return 0;
}
