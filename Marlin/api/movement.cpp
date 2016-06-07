#include "api.h"

#include "../Marlin.h"
#include "../planner.h"
#include "../stepper.h"

static const float s_defaultRetractDistance[3] = {
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

static float s_maxTravelInAxis(int axis) {
  switch(axis) {
    case X_AXIS: return getHomedState(X_AXIS) ? X_MAX_LENGTH : X_MAX_LENGTH_BEFORE_HOMING;
    case Y_AXIS: return getHomedState(Y_AXIS) ? Y_MAX_LENGTH : Y_MAX_LENGTH_BEFORE_HOMING;
    case Z_AXIS: return getHomedState(Z_AXIS) ? Z_MAX_LENGTH : Z_MAX_LENGTH_BEFORE_HOMING;
    default:
      SERIAL_ERROR_START;
      SERIAL_ERROR("Unable to determine max travel distance for axis, axis ");
      SERIAL_ERROR(axis);
      SERIAL_ERROR(" is not recognized - defaulting to a value of 0");
      return 0; // Will likely result in an obvious error, which we can fix
  }
}

int outputMovementStatus() {
  // Position
  SERIAL_ECHO_START;
  SERIAL_ECHO("Position");
  SERIAL_ECHO(" X:"); SERIAL_ECHO(current_position[X_AXIS]);
  SERIAL_ECHO(" Y:"); SERIAL_ECHO(current_position[Y_AXIS]);
  SERIAL_ECHO(" Z:"); SERIAL_ECHO(current_position[Z_AXIS]);
  SERIAL_ECHO(" E:"); SERIAL_ECHO(current_position[E_AXIS]);
  SERIAL_ECHO("\n");

  // Stepper status
  SERIAL_ECHO_START;
  SERIAL_ECHO("StepperPosition");
  SERIAL_ECHO(" x:"); SERIAL_ECHO(st_get_position_mm(X_AXIS));
  SERIAL_ECHO(" y:"); SERIAL_ECHO(st_get_position_mm(Y_AXIS));
  SERIAL_ECHO(" z:"); SERIAL_ECHO(st_get_position_mm(Z_AXIS));
  SERIAL_ECHO(" e:"); SERIAL_ECHO(st_get_position_mm(E_AXIS));
  SERIAL_ECHO("\n");
  SERIAL_ECHO_START;
  SERIAL_ECHO("StepperCounts");
  SERIAL_ECHO(" x:"); SERIAL_ECHO(st_get_position(X_AXIS));
  SERIAL_ECHO(" y:"); SERIAL_ECHO(st_get_position(Y_AXIS));
  SERIAL_ECHO(" z:"); SERIAL_ECHO(st_get_position(Z_AXIS));
  SERIAL_ECHO(" e:"); SERIAL_ECHO(st_get_position(E_AXIS));
  SERIAL_ECHO("\n");

  // Planner
  SERIAL_ECHO_START;
  SERIAL_ECHO("Planner");
  SERIAL_ECHO(" movesPlanned:"); SERIAL_ECHO((int)movesplanned());
  SERIAL_ECHO("\n");

  // Homing state on each axis
  // 0 = not homed, -1 = homed to min extent, 1 = homed to max extent
  SERIAL_ECHO_START;
  SERIAL_ECHO("Homing");
  SERIAL_ECHO(" x:"); SERIAL_ECHO(getHomedState(X_AXIS));
  SERIAL_ECHO(" y:"); SERIAL_ECHO(getHomedState(Y_AXIS));
  SERIAL_ECHO(" z:"); SERIAL_ECHO(getHomedState(Z_AXIS));
  SERIAL_ECHO("\n");

  SERIAL_ECHO_START;
  SERIAL_ECHO("Axis ranges");
  SERIAL_ECHO(" x:"); SERIAL_ECHO(min_pos[X_AXIS]); SERIAL_ECHO(" to "); SERIAL_ECHO(max_pos[X_AXIS]);
  SERIAL_ECHO(" y:"); SERIAL_ECHO(min_pos[Y_AXIS]); SERIAL_ECHO(" to "); SERIAL_ECHO(max_pos[Y_AXIS]);
  SERIAL_ECHO(" z:"); SERIAL_ECHO(min_pos[Z_AXIS]); SERIAL_ECHO(" to "); SERIAL_ECHO(max_pos[Z_AXIS]);
  SERIAL_ECHO("\n");

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

static int s_move(float x, float y, float z, float e, float f) {
  const float speed_in_mm_per_min = f < 0 ? getDefaultFeedrate() : f;

  current_position[ X_AXIS ] = x;
  current_position[ Y_AXIS ] = y;
  current_position[ Z_AXIS ] = z;
  current_position[ E_AXIS ] = e;

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHO("Move");
    SERIAL_ECHO(" X:"); SERIAL_ECHO(current_position[ X_AXIS ]);
    SERIAL_ECHO(" Y:"); SERIAL_ECHO(current_position[ Y_AXIS ]);
    SERIAL_ECHO(" Z:"); SERIAL_ECHO(current_position[ Z_AXIS ]);
    SERIAL_ECHO(" E:"); SERIAL_ECHO(current_position[ E_AXIS ]);
    SERIAL_ECHO(" F:"); SERIAL_ECHO(speed_in_mm_per_min);
    SERIAL_ECHO("\n");
  }

  plan_buffer_line(
    current_position[ X_AXIS ],
    current_position[ Y_AXIS ],
    current_position[ Z_AXIS ],
    current_position[ E_AXIS ],
    speed_in_mm_per_min/60,
    active_extruder
  );
  return 0;
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

static int s_moveIsSafe(float x, float y, float z) {
  bool unsafeInX = s_moveIsUnsafeInAxis( X_AXIS, x );
  bool unsafeInY = s_moveIsUnsafeInAxis( Y_AXIS, y );
  bool unsafeInZ = s_moveIsUnsafeInAxis( Z_AXIS, z );
  return unsafeInX || unsafeInY || unsafeInZ ? -1 : 0;
}

int move(float x, float y, float z , float e, float f) {
  return (
    s_moveIsSafe(x, y, z) ||
    s_move(x, y, z, e, f)
  );
}

int moveXY(float x, float y, float f) {
  return move(x, y, current_position[Z_AXIS], current_position[E_AXIS], f);
}

int moveZ(float z, float f) {
  return move(current_position[X_AXIS], current_position[Y_AXIS], z, current_position[E_AXIS], f);
}

int relativeMove(float x, float y, float z, float e, float speed_in_mm_per_min) {
  return move(
    current_position[ X_AXIS ] + x,
    current_position[ Y_AXIS ] + y,
    current_position[ Z_AXIS ] + z,
    current_position[ E_AXIS ] + e,
    speed_in_mm_per_min
  );
}

int moveToLimit(int axis, int direction, float f, float maxTravel) {

  // Reset the endstop so that we know this movement triggered it (or didn't trigger it)
  clear_endstop(axis);

  // Move
  const auto clampedMaxTravel = min(maxTravel, s_maxTravelInAxis(axis));
  const auto travel = direction < 0 ? -clampedMaxTravel : clampedMaxTravel;
  const float clampedSpeed = f < 0 ? homing_feedrate[axis] : min(f, homing_feedrate[axis]);
  if (s_move(
      current_position[ X_AXIS ] + (axis == X_AXIS ? travel : 0.0f),
      current_position[ Y_AXIS ] + (axis == Y_AXIS ? travel : 0.0f),
      current_position[ Z_AXIS ] + (axis == Z_AXIS ? travel : 0.0f),
      0.0f,
      clampedSpeed
    )) {
    return -1;
  }
  st_synchronize();

  // Confirm we triggered
  if (!endstop_triggered(axis)) { // TODO: weak test, should check specific switches
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to move to ");
    SERIAL_ERROR(direction < 0 ? '-' : '+');
    SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERROR(" limit, limit switch did not trigger\n");
    return -1;
  }

  // Resync with true position
  s_fixPosition(axis);
  return 0;
}

int raise() {
  return moveToLimit(Z_AXIS, 1);
}

int retractFromSwitch(int axis, int direction) {
  // Finish any pending moves (prevents crashes)
  st_synchronize();


  // Retract slightly
  const float retractDistance = s_defaultRetractDistance[axis];
  if(logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHO("Retract by: ");
    SERIAL_ECHO(retractDistance);
    SERIAL_ECHO("\n");
  }
  if (relativeMove(
      axis == X_AXIS ? retractDistance * -direction : 0,
      axis == Y_AXIS ? retractDistance * -direction : 0,
      axis == Z_AXIS ? retractDistance * -direction : 0,
      0
    )) {
    return -1;
  }
  st_synchronize();

  // Confirm that the switch was released
  if (endstop_triggered(axis)) {
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to retract from ");
    SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERROR(" switch, switch did not release during retract movement\n");
    return -1;
  }

  return 0;
}

int measureAtSwitch(int axis, int direction, float maxTravel, float& measurement) {
  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Move to limit
  if (moveToLimit(axis, direction, useDefaultFeedrate, maxTravel) != 0) {
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to measure at ");
    SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERROR(" switch, switch did not trigger during initial approach\n");
    return -1;
  }

  // Retract slightly
  if (retractFromSwitch(axis, direction)) {
    return -1;
  }

  // Approach again, slowly
  // NOTE: this should give us a more accurate reading
  const auto slow = homing_feedrate[axis] / 6;
  const float retractDistance = s_defaultRetractDistance[axis];
  if (moveToLimit(axis, direction, slow, 2 * retractDistance)) {
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to measure at ");
    SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERROR(" switch, switch did not trigger during second approach\n");
    return -1;
  }

  // Record the measurement
  measurement = current_position[axis];
  if(logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHO("Measurement: "); SERIAL_ECHOLN(measurement);
  }
  return 0;
}

