#include "api.h"

#include "../Marlin.h"

int moveToXyPositioner() {
  if (xypos_x_pos != current_position[X_AXIS] || xypos_y_pos != current_position[Y_AXIS]) {
    raise();
  }
  return (
    moveXY(xypos_x_pos, xypos_y_pos) ||
    moveZ(xypos_z_pos)
  );
}

int xyPositionerTouch(int axis, int direction, float& measurement) {
  // Move according to the given axis and direction until a switch is triggered
  const auto slow = homing_feedrate[axis] / 6;
  if (moveToLimit(axis, direction, slow, 5.0f)) {
    return -1;
  }
  measurement = current_position[axis];
  return 0;
}

int xyPositionerFindCenter(long cycles, float& centerX, float& centerY) {
  // Goto the xy positioner
  if (moveToXyPositioner()) {
    return -1;
  }

  // Compute the center
  float measurement1;
  float measurement2;
  centerX = xypos_x_pos;
  centerY = xypos_y_pos;
  for (int i=0; i<cycles; ++i) {
    // Compute center X
    if ( moveXY(centerX, centerY) // applies new centerY on 2nd iteration
      || xyPositionerTouch(X_AXIS, 1, measurement1) // measure +x
      || moveXY(centerX, centerY) // recenter
      || xyPositionerTouch(X_AXIS, -1, measurement2)) { // measure -x
      return -1;
    }
    centerX = (measurement2 + measurement1) / 2;

    if (logging_enabled){
      SERIAL_ECHO_START;
      SERIAL_ECHO("xyPositionerCenterX");
      SERIAL_ECHO(" m1:"); SERIAL_ECHO(measurement1);
      SERIAL_ECHO(" m2:"); SERIAL_ECHO(measurement2);
      SERIAL_ECHO(" x:"); SERIAL_ECHO(centerX);
      SERIAL_ECHO("\n");
    }

    // Compute center Y
    if ( moveXY(centerX, centerY) // applies new centerX
      || xyPositionerTouch(Y_AXIS, 1, measurement1) // measure +y
      || moveXY(centerX, centerY) // recenter
      || xyPositionerTouch(Y_AXIS, -1, measurement2)) { // measure -y
      return -1;
    }
    centerY = (measurement2 + measurement1) / 2;

    if (logging_enabled){
      SERIAL_ECHO_START;
      SERIAL_ECHO("xyPositionerCenterY");
      SERIAL_ECHO(" m1:"); SERIAL_ECHO(measurement1);
      SERIAL_ECHO(" m2:"); SERIAL_ECHO(measurement2);
      SERIAL_ECHO(" y:"); SERIAL_ECHO(centerY);
      SERIAL_ECHO("\n");
    }

    // Each cycle takes a non-trivial amount of time so reset the inactivity timer
    refresh_cmd_timeout();
  }

  // Go to the computed position
  if (moveXY(centerX, centerY)) {
    return -1;
  }

  return 0;
}
