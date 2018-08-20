#include "../../../Axis.h"
#include "../../../planner.h"

#include "../../../MarlinConfig.h"
#include "../../compensationAlgorithms/api.h"
#include "../../utils/ScopedInterruptDisable.h"
#include "../../api/movement/movement.h" // mm to steps


float stepsToPositionInAxis(AxisEnum axis, long stepCounts[NUM_AXIS]) {
  // Need to apply scaling and skew compenstion in reverse
  // DEFER: define reverseSkew and reverseScaling
  switch(axis) {
    case X_AXIS:
      return reverseSkewCompensationInX(
        reverseScalingCompensation(
          stepsToMillimeters(stepCounts[X_AXIS], X_AXIS), calib_x_scale
        ),
        calib_cos_theta
      );

    case Y_AXIS: {
      return reverseSkewCompensationInY(
        reverseScalingCompensation(
          stepsToMillimeters(stepCounts[X_AXIS], X_AXIS), calib_x_scale
        ),
        reverseScalingCompensation(
          stepsToMillimeters(stepCounts[Y_AXIS], Y_AXIS), calib_y_scale
        ),
        calib_tan_theta
      );
    }

    case Z_AXIS:
    case E_AXIS:
      return stepsToMillimeters(stepCounts[axis], axis);
  }
  return 0;
}

float stepsToPositionInAxis(AxisEnum axis, volatile long stepCounts[NUM_AXIS]) {
  long copiedSteps[NUM_AXIS];
  {
    ScopedInterruptDisable sid;
    copiedSteps[X_AXIS] = stepCounts[X_AXIS];
    copiedSteps[Y_AXIS] = stepCounts[Y_AXIS];
    copiedSteps[Z_AXIS] = stepCounts[Z_AXIS];
    copiedSteps[E_AXIS] = stepCounts[E_AXIS];
  }
  return stepsToPositionInAxis(axis, copiedSteps);
}
