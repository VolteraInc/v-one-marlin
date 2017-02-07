#include "api.h"

#include "../../Marlin.h"

int moveToXyPositioner(Tool tool, bool skipMoveInZ) {
  if(logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Move to xy positioner");
  }

  switch(tool) {
    case TOOLS_DISPENSER:
    case TOOLS_PROBE:
      break;
    default:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to move to xy-positioner, no tool attached");
      return -1;
  }

  // Raise, unless we are really close to the target x,y position
  auto const dx = abs(xypos_x_pos - current_position[X_AXIS]);
  auto const dy = abs(xypos_y_pos - current_position[Y_AXIS]);
  if (dx > 1 || dy > 1) {
    if (raise()) {
      return -1;
    }
  }

  // Move to xy-positioner's x,y
  if (moveXY(tool, xypos_x_pos, xypos_y_pos)) {
    return -1;
  }

  // move to xy-positioner's z, unless told to skip
  if (!skipMoveInZ && moveZ(tool, XYPOS_Z_POS)) {
    return -1;
  }

  return 0;
}

int xyPositionerTouch(Tool tool, int axis, int direction, float& measurement) {
  switch(tool) {
    case TOOLS_DISPENSER:
    case TOOLS_PROBE:
      break;
    default:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to touch xy-positioner switches, no tool attached");
      return -1;
  }

  // Move according to the given axis and direction until a switch is triggered
  const auto slow = homing_feedrate[axis] / 6;
  if (moveToLimit(axis, direction, slow, 5.0f)) {
    return -1;
  }
  measurement = current_position[axis];
  return 0;
}

static int s_findCenter(Tool tool, long cycles, float& o_centerX, float& o_centerY) {
  if(logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Find center of xy positioner");
  }

  // Compute the center
  float measurement1;
  float measurement2;
  auto centerX = xypos_x_pos;
  auto centerY = xypos_y_pos;
  for (int i=0; i<cycles; ++i) {
    // Compute center X
    if ( moveXY(tool, centerX, centerY) // applies new centerY on 2nd iteration
      || xyPositionerTouch(tool, X_AXIS, 1, measurement1) // measure +x
      || moveXY(tool, centerX, centerY) // recenter
      || xyPositionerTouch(tool, X_AXIS, -1, measurement2)) { // measure -x
      return -1;
    }
    centerX = (measurement2 + measurement1) / 2;

    if (logging_enabled){
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("xyPositionerCenterX");
      SERIAL_ECHOPGM(" m1:"); SERIAL_ECHO(measurement1);
      SERIAL_ECHOPGM(" m2:"); SERIAL_ECHO(measurement2);
      SERIAL_ECHOPGM(" x:"); SERIAL_ECHO(centerX);
      SERIAL_ECHOPGM("\n");
    }

    // Compute center Y
    if ( moveXY(tool, centerX, centerY) // applies new centerX
      || xyPositionerTouch(tool, Y_AXIS, 1, measurement1) // measure +y
      || moveXY(tool, centerX, centerY) // recenter
      || xyPositionerTouch(tool, Y_AXIS, -1, measurement2)) { // measure -y
      return -1;
    }
    centerY = (measurement2 + measurement1) / 2;

    if (logging_enabled){
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("xyPositionerCenterY");
      SERIAL_ECHOPGM(" m1:"); SERIAL_ECHO(measurement1);
      SERIAL_ECHOPGM(" m2:"); SERIAL_ECHO(measurement2);
      SERIAL_ECHOPGM(" y:"); SERIAL_ECHO(centerY);
      SERIAL_ECHOPGM("\n");
    }

    // Each cycle takes a non-trivial amount of time so reset the inactivity timer
    refresh_cmd_timeout();
  }

  // Go to the computed position
  if (moveXY(tool, centerX, centerY)) {
    return -1;
  }

  // Success
  o_centerX = centerX;
  o_centerY = centerY;
  return 0;
}

int xyPositionerFindCenter(Tool tool, long cycles, float& centerX, float& centerY) {
  return (
    moveToXyPositioner(tool) ||
    s_findCenter(tool, cycles, centerX, centerY)
  );
}

int calibrateKeyPositions(Tool tool, long cycles) {
  // Make sure we have a probe
  if (tool != TOOLS_PROBE) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to calibrate positions, probe not attached");
    return -1;
  }

  // Home in X and Y
  // Ensure homed in X, Y
  if (!homedXY()) {
    if (homeXY()) {
      return -1;
    }
  }

  // Move to the x,y location of the xy-positioner's
  // Notes:
  //   - We use the hardcoded default position, not the stored values, which could be wrong.
  //   - We skip the Z movement because we have not homed Z. We don't home Z because
  //     we'd need a reliable hardcoded x,y position for the z-switch, which has proven difficult.
  //     Meanwhile, the xy-position is more tolerant of inaccuracies in the hardcoded values.
  if (moveToXyPositioner(tool, skipMoveInZ)) {
    return -1;
  }

  // Lower in Z (probe switch should trigger)
  if (moveToLimit(Z_AXIS, -1)) {
    return -1;
  }

  // Retract slightly
  if (relativeMove(tool, 0, 0, 2, 0)) {
    return -1;
  }

  // Find the center
  float centerX;
  float centerY;
  if (s_findCenter(tool, cycles, centerX, centerY)) {
    return -1;
  }

  // Set the x,y position of the z-switch using hardcoded offset values.
  min_z_x_pos = centerX + OFFSET_FROM_XYPOS_TO_MINZ_X;
  min_z_y_pos = centerY + OFFSET_FROM_XYPOS_TO_MINZ_Y;

  // Home Z (uses the new z-switch location)
  if (homeZ(tool)) {
    return -1;
  }

  // Find the center, using the standard algorithm
  // Note: This will use a hardcoded Z position (unlike the previous call)
  if (xyPositionerFindCenter(tool, cycles, centerX, centerY)) {
    return -1;
  }

  // Set the x,y position of the xy-positioner
  xypos_x_pos = centerX;
  xypos_y_pos = centerY;

  // Success
  return 0;
}
