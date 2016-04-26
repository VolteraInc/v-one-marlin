#include "api.h"

#include "../Marlin.h"
#include "../planner.h"
#include "../stepper.h"

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
}

static float s_maxTravelInAxis(int axis) {
  switch(axis) {
    case X_AXIS: return X_MAX_LENGTH;
    case Y_AXIS: return Y_MAX_LENGTH;
    case Z_AXIS: return Z_MAX_LENGTH;
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
  SERIAL_PROTOCOL("Position");
  SERIAL_PROTOCOL(" X:"); SERIAL_PROTOCOL_F(current_position[X_AXIS], 6);
  SERIAL_PROTOCOL(" Y:"); SERIAL_PROTOCOL_F(current_position[Y_AXIS], 6);
  SERIAL_PROTOCOL(" Z:"); SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
  SERIAL_PROTOCOL(" E:"); SERIAL_PROTOCOL_F(current_position[E_AXIS], 6);
  SERIAL_PROTOCOL("\n");

  // Stepper status
  SERIAL_PROTOCOL("StepperPosition");
  SERIAL_PROTOCOL(" x:"); SERIAL_PROTOCOL_F(st_get_position_mm(X_AXIS), 6);
  SERIAL_PROTOCOL(" y:"); SERIAL_PROTOCOL_F(st_get_position_mm(Y_AXIS), 6);
  SERIAL_PROTOCOL(" z:"); SERIAL_PROTOCOL_F(st_get_position_mm(Z_AXIS), 6);
  SERIAL_PROTOCOL(" e:"); SERIAL_PROTOCOL_F(st_get_position_mm(E_AXIS), 6);
  SERIAL_PROTOCOL("\n");
  SERIAL_PROTOCOL("StepperCounts");
  SERIAL_PROTOCOL(" x:"); SERIAL_PROTOCOL(st_get_position(X_AXIS));
  SERIAL_PROTOCOL(" y:"); SERIAL_PROTOCOL(st_get_position(Y_AXIS));
  SERIAL_PROTOCOL(" z:"); SERIAL_PROTOCOL(st_get_position(Z_AXIS));
  SERIAL_PROTOCOL(" e:"); SERIAL_PROTOCOL(st_get_position(E_AXIS));
  SERIAL_PROTOCOL("\n");

  // Planner
  SERIAL_PROTOCOL("Planner");
  SERIAL_PROTOCOL(" movesPlanned:"); SERIAL_PROTOCOL((int)movesplanned());
  SERIAL_PROTOCOL("\n");

  // Homing state on each axis
  // 0 = not homed, -1 = homed to min extent, 1 = homed to max extent
  SERIAL_PROTOCOL("Homing");
  SERIAL_PROTOCOL(" x:"); SERIAL_PROTOCOL(getHomedState(X_AXIS));
  SERIAL_PROTOCOL(" y:"); SERIAL_PROTOCOL(getHomedState(Y_AXIS));
  SERIAL_PROTOCOL(" z:"); SERIAL_PROTOCOL(getHomedState(Z_AXIS));
  SERIAL_PROTOCOL("\n");
  return 0;
}

float getDefaultFeedrate() {
  // Returns the max feedrate across all axes
  return max(
    max(
      max_feedrate[X_AXIS],
      max_feedrate[Y_AXIS]
    ),
    max_feedrate[Z_AXIS]
  );
}

int move(float x, float y, float z, float e, float speed_in_mm_per_min) {
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

int moveXY(float x, float y, float f) {
  const float feedrate = f < 0 ? getDefaultFeedrate() : f;
  return move(x, y, current_position[Z_AXIS], current_position[E_AXIS], feedrate);
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

int moveToLimit(int axis, int direction, float speed_in_mm_per_min, float maxTravel) {
  const auto clampedSpeed = min(speed_in_mm_per_min, homing_feedrate[axis]);
  const auto clampedMaxTravel = min(maxTravel, s_maxTravelInAxis(axis));
  const auto travel = direction > 0 ? clampedMaxTravel : -clampedMaxTravel;
  relativeMove(
    axis == X_AXIS ? travel : 0.0f,
    axis == Y_AXIS ? travel : 0.0f,
    axis == Z_AXIS ? travel : 0.0f,
    0.0f,
    clampedSpeed
  );
  st_synchronize();

  if (!didHitEndstops()) { // TODO: weak test, should check the specific switch
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to move to ");
    SERIAL_ERROR(direction < 0 ? '-' : '+');
    SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERROR(" limit, limit switch did not trigger\n");
    return -1;
  }

  s_fixPosition(axis);
  endstops_hit_on_purpose();
  return 0;
}

int probe(float retractDistance, float& measurement) {
  float fast = homing_feedrate[Z_AXIS];
  float slow = homing_feedrate[Z_AXIS] / 6.0;

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Move down until you find the bed
  if (moveToLimit(Z_AXIS, -1, fast, Z_MAX_LENGTH) != 0) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to collect probe measurement, probe switch did not trigger during initial decent");
    return -1;
  }

  // Move up by the retract distance
  relativeMove(0, 0, retractDistance, 0, fast);
  st_synchronize();

  // Move back down slowly to find bed
  // NOTE: should give us a more accurate reading
  if (moveToLimit(Z_AXIS, -1, slow, 2 * retractDistance) != 0) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to collect probe measurement, probe switch did not trigger during second decent");
    return -1;
  }

  measurement = current_position[Z_AXIS];
  return 0;
}
