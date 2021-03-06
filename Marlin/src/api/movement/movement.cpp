#include "../api.h"

#include "../../../Marlin.h"
#include "../../../planner.h"
#include "../../../stepper.h"

#include "../../vone/VOne.h"
#include "../../vone/endstops/Endstop.h"
#include "../../vone/endstops/ScopedEndstopEnable.h"

static bool s_establishedSoftMaxZ = false;

static const float s_defaultRetractDistance[] = {
  X_HOME_RETRACT_MM,
  Y_HOME_RETRACT_MM,
  Z_HOME_RETRACT_MM
};

static float s_maxTravelInAxis(AxisEnum axis, int direction) {
  switch(axis) {
    case X_AXIS: return getHomedState(X_AXIS) ? X_MAX_LENGTH + 1 : X_MAX_LENGTH_BEFORE_HOMING;
    case Y_AXIS: return getHomedState(Y_AXIS) ? Y_MAX_LENGTH + 1 : Y_MAX_LENGTH_BEFORE_HOMING;
    case Z_AXIS: return getHomedState(Z_AXIS) ? Z_MAX_LENGTH + 1 : (direction < 0 ? Z_MAX_TRAVEL_DOWN_BEFORE_HOMING : Z_MAX_TRAVEL_UP_BEFORE_HOMING);
    default:
      logError
        << F("Unable to determine max travel distance for axis, axis ") << axis
        << F(" is not recognized - defaulting to a value of 0")
        << endl;
      return 0; // Will likely result in an obvious error, which we can fix
  }
}

bool establishedSoftMaxZ() {
  return s_establishedSoftMaxZ;
}

void establishSoftMaxZ(float value) {
  log << F("setting maximum Z position to ") << value << endl;
  s_establishedSoftMaxZ = true;
  max_pos[Z_AXIS] = value;
}

void clearSoftMaxZ() {
  log << F("resetting max z to default") << Z_MAX_POS << endl;
  s_establishedSoftMaxZ = false;
  max_pos[Z_AXIS] = Z_MAX_POS;
}

int outputMovementStatus() {
  // Position
  log
    << F("Position")
    << F(" X:") << current_position[X_AXIS]
    << F(" Y:") << current_position[Y_AXIS]
    << F(" Z:") << current_position[Z_AXIS]
    << F(" E:") << current_position[E_AXIS]
    << endl;

  // Stepper status
  log
    << F("StepperPosition")
    << F(" x:") << st_get_position_mm(X_AXIS)
    << F(" y:") << st_get_position_mm(Y_AXIS)
    << F(" z:") << st_get_position_mm(Z_AXIS)
    << F(" e:") << st_get_position_mm(E_AXIS)
    << endl;
  log
    << F("StepperCounts")
    << F(" x:") << st_get_position(X_AXIS)
    << F(" y:") << st_get_position(Y_AXIS)
    << F(" z:") << st_get_position(Z_AXIS)
    << F(" e:") << st_get_position(E_AXIS)
    << endl;

  // Planner
  log
    << F("Planner")
    << F(" movesPlanned:") << (int)movesplanned()
    << endl;

  // Homing state on each axis
  // 0 = not homed, -1 = homed to min extent, 1 = homed to max extent
  log
    << F("Homing")
    << F(" x:") << getHomedState(X_AXIS)
    << F(" y:") << getHomedState(Y_AXIS)
    << F(" z:") << getHomedState(Z_AXIS)
    << endl;

  log
    << F("Axis ranges")
    << F(" x:") << min_pos[X_AXIS] << F(" to ") << max_pos[X_AXIS]
    << F(" y:") << min_pos[Y_AXIS] << F(" to ") << max_pos[Y_AXIS]
    << F(" z:") << min_pos[Z_AXIS] << F(" to ") << max_pos[Z_AXIS]
    << (establishedSoftMaxZ() ? F(" (using soft max z)") : F(" (using default max z)"))
    << endl;

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

static bool s_moveIsUnsafeInAxis(AxisEnum axis, float value) {
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
    logError
      << F("Unable to move to ") << value
      << F(" in ") << axis_codes[axis]
      << F("-axis, position falls outside of safe bounds")
      << endl;
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
    log
      << F("Move")
      << F(" X:") << current_position[ X_AXIS ]
      << F(" Y:") << current_position[ Y_AXIS ]
      << F(" Z:") << current_position[ Z_AXIS ]
      << F(" E:") << current_position[ E_AXIS ]
      << F(" F:") << speed_in_mm_per_min
      << endl;
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

int moveToLimit(AxisEnum axis, int direction, float f, float maxTravel) {
  auto& endstopMonitor = vone->stepper.endstopMonitor;
  if(logging_enabled) {
    log
      << F("Move to limit: ")
      << (direction < 0 ? '-' : '+')
      << axis_codes[axis]
      << endl;
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Check if already triggered
  if (endstopMonitor.isTriggered(axis, direction)) {
    return 0;
  }

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
  if (endstopMonitor.acknowledgeTriggered(axis, direction)) {
    logError
      << F("Unable to move to ")
      << (direction < 0 ? '-' : '+')
      << axis_codes[axis]
      << F(" limit, limit switch did not trigger")
      << endl;
    return -1;
  }

  // Resync with stepper position
  return vone->stepper.resyncWithStepCount(axis);
}

int moveToEndstop(const Endstop& endstop, float f, float maxTravel) {
  auto& endstopMonitor = vone->stepper.endstopMonitor;
  const auto axis = endstop.axis;
  const auto direction = endstop.direction;

  if (logging_enabled) {
    log << F("Move to end-stop: ") << endstop.name << endl;
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Check if already triggered
  if (endstop.readTriggered()) {
    return 0;
  }

  // Enable endstop, if necessary
  ScopedEndstopEnable scopedEnable(endstopMonitor, endstop);

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
  if (endstopMonitor.acknowledgeTriggered(endstop)) {
    logError
      << F("Unable to move to ")
      << endstop.name
      << F(", switch did not trigger")
      << endl;
    return -1;
  }

  // Resync with stepper position
  return vone->stepper.resyncWithStepCount(axis);
}


int raiseToEndstop() {
  return moveToEndstop(vone->endstops.zMax);
}

int raiseToSoftMax(tools::Tool& tool) {
  if (!establishedSoftMaxZ()) {
    logError
      << F("Unable to move to raise to max height, current tool has not been homed in z")
      << endl;
    return -1;
  }
  return moveZ(tool, max_pos[Z_AXIS]);
}

int raise(tools::Tool& tool) {
  if (!establishedSoftMaxZ()) {
    return raiseToEndstop();
  }
  return raiseToSoftMax(tool);
}

int retractFromSwitch(const Endstop& endstop, float retractDistance) {
  if (logging_enabled) {
    log
      << F("Retract from ")
      << endstop.name
      << endl;
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Retract slightly
  const auto axis = endstop.axis;
  const auto direction = endstop.direction;
  const float distance = retractDistance < 0 ? s_defaultRetractDistance[axis] : retractDistance;
  if (logging_enabled) {
    log << F("Retract by: ") << distance << endl;
  }

  if (axis == E_AXIS) {
    if(relativeRawMoveE(distance * direction, homing_feedrate[E_AXIS])){
      return -1;
    }
  } else {
    if (relativeRawMoveXYZ(
      axis == X_AXIS ? distance * -direction : 0,
      axis == Y_AXIS ? distance * -direction : 0,
      axis == Z_AXIS ? distance * -direction : 0
    )) {
      return -1;
    }
  }

  // Confirm that the switch was released
  // Note: we check the switch itself, we can not
  //       rely on the stepper's endstop monitor
  //       becuase it will not check switches when
  //       moving away from them.
  if (endstop.readTriggered()) {
    logError
      << F("Unable to retract from ")
      << endstop.name
      << F(", switch did not release during retract movement")
      << endl;
    return -1;
  }

  return 0;
}
