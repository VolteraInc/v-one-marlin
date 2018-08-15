#include "../../../Axis.h"
#include "../../../planner.h"

#include "../../../MarlinConfig.h"
#include "../../compensationAlgorithms/api.h"
#include "../../utils/ScopedInterruptDisable.h"

static float s_stepToPositionInAxis(AxisEnum axis, long stepCounts[NUM_AXIS]) {
  return stepCounts[axis] / axis_steps_per_unit[axis];
}

float stepsToPositionInAxis(AxisEnum axis, long stepCounts[NUM_AXIS]) {
  // Need to apply scaling and skew compenstion in reverse
  // DEFER: define reverseSkew and reverseScaling
  switch(axis) {
    case X_AXIS:
      return reverseSkewCompensationInX(
        reverseScalingCompensation(
          s_stepToPositionInAxis(X_AXIS, stepCounts), calib_x_scale
        ),
        calib_cos_theta
      );

    case Y_AXIS: {
      return reverseSkewCompensationInY(
        reverseScalingCompensation(
          s_stepToPositionInAxis(X_AXIS, stepCounts), calib_x_scale
        ),
        reverseScalingCompensation(
          s_stepToPositionInAxis(X_AXIS, stepCounts), calib_y_scale
        ),
        calib_tan_theta
      );
    }

    case Z_AXIS:
    case E_AXIS:
      return stepCounts[axis] / axis_steps_per_unit[axis];
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
