#include "api.h"

#include "../../serial.h"
#include "../../Marlin.h"
#include "../work/work.h"
#include "../vone/VOne.h"
#include "../vone/endstops/Endstop.h"
#include "../vone/endstops/ScopedEndstopEnable.h"

const float defaultXyPositionerCycles = 2;

static int s_moveToXyPositionerZ(tools::Tool& tool, enum HowToMoveToZ howToMoveToZ) {
  switch (howToMoveToZ) {
    case useConfiguredZ:
      return moveZ(tool, xypos_z_pos);

    case usePlateBackOffForZ: {
      const auto& calibrationPlate = vone->endstops.calibrationPlate;
      return (
        // Lower until plate triggers
        moveToEndstop(calibrationPlate) ||

        // Retract .5mm above surface
        // Note: probe displacement can be ignored becuase the
        //       calibration plate triggers before the probe
        relativeMove(tool, 0, 0, .5, 0)
      );
    }

    case skipMoveInZ:
      return 0;

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
    if (raise(tool)) {
      return -1;
    }
  }

  // Move to xy-positioner's x,y, then z
  return (
    moveXY(tool, xypos_x_pos, xypos_y_pos) ||
    s_moveToXyPositionerZ(tool, howToMoveToZ)
  );
}

int xyPositionerTouch(tools::Tool& tool, const Endstop& endstop, float& measurement) {
  const auto axis = endstop.axis;
  const auto slow = homing_feedrate[axis] / 6;
  auto& endstopMonitor = vone->stepper.endstopMonitor;
  const auto& yMin = vone->endstops.yMin;
  const auto& xyPositionerBack = vone->endstops.xyPositionerBack;

  if (&endstop != &xyPositionerBack) {
    if (moveToEndstop(endstop, slow, 6.0f)) {
      return -1;
    }
  } else {
    // Enable xyBack
    // Note: moveToEndstop will do this automatically,
    //       but moveToLimit does not
    ScopedEndstopEnable scopedEnable(endstopMonitor, endstop);

    // Note: 'moveToLimit' does not differentiate between switches
    if (moveToLimit(axis, endstop.direction, slow, 6.0f)) {
      return -1;
    }

    // Warn if yMin triggered instead of xyBack
    // Note: eventually this will be made an error
    if (
      !endstopMonitor.isTriggered(xyPositionerBack) &&
      endstopMonitor.isTriggered(yMin)
    ) {
      logWarning
        << F("The ") << yMin.name << F(" switch triggered before reaching ")
        << F("the ") << xyPositionerBack.name << F(" switch -- ")
        << F("the current tool is a ") << tool.name()
        << endl;
    }
  }

  // Success
  measurement = current_position[axis];
  return 0;
}


static int s_findCenter(tools::Tool& tool, long cycles, float& o_centerX, float& o_centerY) {
  const auto& endstops = vone->endstops;
  if(logging_enabled) {
    log << F("Find center of xy positioner") << endl;
  }

  // Compute the center
  float measurement1;
  float measurement2;
  auto centerX = xypos_x_pos;
  auto centerY = xypos_y_pos;
  for (int i=0; i<cycles; ++i) {
    // Measure left and right
    if ( moveXY(tool, centerX, centerY) // applies new centerY on 2nd iteration
      || xyPositionerTouch(tool, endstops.xyPositionerLeft, measurement1) // measure +x
      || relativeMove(tool, -5.0f, 0, 0, 0)
      || xyPositionerTouch(tool, endstops.xyPositionerRight, measurement2)) { // measure -x
      return -1;
    }

    // Compute X center
    centerX = (measurement2 + measurement1) / 2;
    log
      << F("xyPositionerCenterX")
      << F(" m1:") << measurement1
      << F(" m2:") << measurement2
      << F(" x:") << centerX
      << endl;

    // Measure forward and back
    if ( moveXY(tool, centerX, centerY) // applies new centerX
      || xyPositionerTouch(tool, endstops.xyPositionerForward, measurement1) // measure +y
      || relativeMove(tool, 0, -5.0f, 0, 0)
      || xyPositionerTouch(tool, endstops.xyPositionerBack, measurement2)) { // measure -y
      return -1;
    }

    // Compute Y center
    centerY = (measurement2 + measurement1) / 2;
    log
      << F("xyPositionerCenterY")
      << F(" m1:") << measurement1
      << F(" m2:") << measurement2
      << F(" y:") << centerY
      << endl;

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
