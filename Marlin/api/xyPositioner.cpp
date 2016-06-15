#include "api.h"

#include "../Marlin.h"

int moveToXyPositioner(Tool tool) {
  // Make sure we have a probe
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

  // Move to xy-positioner
  return (
    moveXY(tool, xypos_x_pos, xypos_y_pos) ||
    moveZ(tool, XYPOS_Z_POS, useDefaultFeedrate, ignoreDispenseHeight)
  );
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

int xyPositionerFindCenter(Tool tool, long cycles, float& centerX, float& centerY) {
  // Goto the xy positioner
  if (moveToXyPositioner(tool)) {
    return -1;
  }

  // Compute the center
  float measurement1;
  float measurement2;
  centerX = xypos_x_pos;
  centerY = xypos_y_pos;
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

  return 0;
}
