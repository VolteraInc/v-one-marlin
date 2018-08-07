#include "api.h"

#include "../../Marlin.h"
#include "../work/work.h"

static int s_moveToXyPositionerZ(tools::Tool& tool, enum HowToMoveToZ howToMoveToZ) {
  switch (howToMoveToZ) {
    case useConfiguredZ:
      return moveZ(tool, XYPOS_Z_POS);

    case usePlateBackOffForZ:
      return (
        moveToLimit(Z_AXIS, -1) ||      // Lower in Z (probe switch should trigger)
        relativeMove(tool, 0, 0, 2, 0)  // Retract slightly
      );

    case skipMoveInZ:
      return;

    default:
      logError << F("Unable to move to xy-positioner, unrecognized arguments") << endl;
      return -1;
  }
}

int moveToXyPositioner(tools::Tool& tool, enum HowToMoveToZ howToMoveToZ) {
  log << F("Move to xy positioner") << endl;

  // Raise, unless we are really close to the target x,y position
  auto const dx = abs(xypos_x_pos - current_position[X_AXIS]);
  auto const dy = abs(xypos_y_pos - current_position[Y_AXIS]);
  if (dx > 1 || dy > 1) {
    if (raise()) {
      return -1;
    }
  }

  // Move to xy-positioner's x,y, then z
  return (
    moveXY(tool, xypos_x_pos, xypos_y_pos) ||
    s_moveToXyPositionerZ(tool, howToMoveToZ)
  );
}

int xyPositionerTouch(tools::Tool&, int axis, int direction, float& measurement) {
  // Move according to the given axis and direction until a switch is triggered
  const auto slow = homing_feedrate[axis] / 6;
  if (moveToLimit(axis, direction, slow, 6.0f)) {
    return -1;
  }
  measurement = current_position[axis];
  return 0;
}

static int s_findCenter(tools::Tool& tool, long cycles, float& o_centerX, float& o_centerY) {
  if(logging_enabled) {
    log << F("Find center of xy positioner") << endl;
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
      || relativeMove(tool, -5.0f, 0, 0, 0)
      || xyPositionerTouch(tool, X_AXIS, -1, measurement2)) { // measure -x
      return -1;
    }
    centerX = (measurement2 + measurement1) / 2;

    if (logging_enabled){
      log
        << F("xyPositionerCenterX")
        << F(" m1:") << measurement1
        << F(" m2:") << measurement2
        << F(" x:") << centerX
        << endl;
    }

    // Compute center Y
    if ( moveXY(tool, centerX, centerY) // applies new centerX
      || xyPositionerTouch(tool, Y_AXIS, 1, measurement1) // measure +y
      || relativeMove(tool, 0, -5.0f, 0, 0)
      || xyPositionerTouch(tool, Y_AXIS, -1, measurement2)) { // measure -y
      return -1;
    }
    centerY = (measurement2 + measurement1) / 2;

    if (logging_enabled){
      log
        << F("xyPositionerCenterY")
        << F(" m1:") << measurement1
        << F(" m2:") << measurement2
        << F(" y:") << centerY
        << endl;
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

int xyPositionerFindCenter(tools::Tool& tool, long cycles, float& centerX, float& centerY, enum HowToMoveToZ howToMoveToZ) {
  return (
    moveToXyPositioner(tool, howToMoveToZ) ||
    s_findCenter(tool, cycles, centerX, centerY)
  );
}
