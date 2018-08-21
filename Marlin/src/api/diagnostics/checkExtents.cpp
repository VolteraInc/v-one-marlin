#include "../../api/api.h"
#include "../../../Marlin.h"
#include "../../vone/VOne.h"

static int s_checkMeasurements(
  const __FlashStringHelper* context,
  tools::Tool& tool,
  const Endstop& endstop,
  float measurement1,
  float measurement2,
  float tolerance
) {
  if (tolerance < 0) {
    // Use 1 step by default + 1um to handle numerial error
    // Note: 1 step should be 10um
    tolerance = stepsToMillimeters(1, endstop.axis) + 0.001;
  }
  const auto delta = fabs(measurement2 - measurement1);

  // Log data
  log
    << context
    << F(", m1=") << measurement1
    << F(", m2=") << measurement2
    << F(", delta=") << delta
    << endl;

  // Check measurements
  if (delta > tolerance) {
    const auto deltaInSteps = millimetersToSteps(delta, endstop.axis);
    logError
      << F("Unable to complete check of ") << context
      << F(", measurements = [") << measurement1 << F(", ") << measurement2 << F("] ")
      << F("differ by ") << delta << F("mm (~") << deltaInSteps << F("steps) ")
      << F("which is more than the ") << tolerance << F("mm tolerance")
      << endl;

    // Failure means we hit a physical limit which
    // means we need to re-zero the axis
    if (tool.resetPreparations()) {
      logWarning
        << "Unable to reset tool preparations after extents test failed, ignoring"
        << endl;
    }

    return -1;
  }

  // Success
  return 0;
}

static int s_checkExtent(
  const __FlashStringHelper* context,
  tools::Tool& tool,
  const Endstop& endstop,
  const Point2d& start,
  const Point2d& end,
  float tolerance
) {
  float measurement1;
  float measurement2;
  return (
    tool.prepareToMove() ||
    raise() ||

    // Measure at min_pos
    moveXY(tool, min_pos[X_AXIS], min_pos[Y_AXIS]) ||
    measureAtSwitch(endstop, useDefaultMaxTravel, measurement1) ||

    // Move to start, then to end
    // Note: should not loose any steps
    moveXY(tool, start.x, start.y) ||
    moveXY(tool, end.x, end.y) ||
    moveXY(tool, start.x, start.y) ||

    // Measure again
    moveXY(tool, min_pos[X_AXIS], min_pos[Y_AXIS]) ||
    measureAtSwitch(endstop, useDefaultMaxTravel, measurement2) ||

    // Check measurements
    s_checkMeasurements(context, tool, endstop, measurement1, measurement2, tolerance)
  );
}

int checkExtents(tools::Tool& tool, float tolerance) {
  Point2d backLeft   = { max_pos[X_AXIS], min_pos[Y_AXIS] };
  Point2d backRight  = { min_pos[X_AXIS], min_pos[Y_AXIS] };
  Point2d frontLeft  = { max_pos[X_AXIS], max_pos[Y_AXIS] };
  Point2d frontRight = { min_pos[X_AXIS], max_pos[Y_AXIS] };

  int returnValue = (
    // x-axis extents
    s_checkExtent(F("X-axis extent - across back" ), tool, vone->endstops.xMin,  backRight,   backLeft, tolerance) ||
    s_checkExtent(F("X-axis extent - across front"), tool, vone->endstops.xMin, frontRight, frontLeft, tolerance) ||

    // y-axis extents
    s_checkExtent(F("Y-axis extent - along right-side"), tool, vone->endstops.yMin, backRight, frontRight, tolerance) ||
    s_checkExtent(F("Y-axis extent - along left-side" ), tool, vone->endstops.yMin, backLeft,  frontLeft,  tolerance) ||

    // Diagonal to front left
    s_checkExtent(F("X-axis extent - diagonal to front left"), tool, vone->endstops.xMin, backRight, frontLeft, tolerance) ||
    s_checkExtent(F("Y-axis extent - diagonal to front left"), tool, vone->endstops.yMin, backRight, frontLeft, tolerance) ||

    // Diagonal to front right
    s_checkExtent(F("X-axis extent - diagonal to front right"), tool, vone->endstops.xMin, backLeft, frontRight, tolerance) ||
    s_checkExtent(F("Y-axis extent - diagonal to front right"), tool, vone->endstops.yMin, backLeft, frontRight, tolerance)
  );

  // Note: Touching switches and resycning with the stepper
  //       can introduce a small error, we don't want that
  //       error impacting any commands run after this one.
  setHomedState(X_AXIS, 0);
  setHomedState(Y_AXIS, 0);
  setHomedState(Z_AXIS, 0);
  return (
    tool.resetPreparations() ||
    returnValue
  );
}