#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"

extern float axis_steps_per_unit[4];
static float s_probeDisplacement = 0.0f;

int s_partiallyPrepareProbe(const char* context, Tool tool) {
  enable_p_top(true);
  return (
    raise() ||
    confirmMountedAndNotTriggered(context, tool, TOOLS_PROBE) ||
    ensureHomedInXY()
  );
}

int prepareProbe(Tool tool) {
  const char* context = "prepare probe";
  return (
    s_partiallyPrepareProbe(context, tool) ||
    homeZ(tool) || // home Z so we can enter the xy pos with decent precision
    centerTool(tool) ||
    measureProbeDisplacement(tool, s_probeDisplacement) || // do this now, saves a trip back to the xy-pos after re-homing
    homeZ(tool) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)
    raise()
  );
}

int s_recordMeasurement(float& measurement) {
  // the point of contact is 1 step above the current height
  // This is becuase the stepper needs 2 steps to detect a hit
  const float height = current_position[Z_AXIS] + 1 / axis_steps_per_unit[Z_AXIS];
  measurement = height + s_probeDisplacement;
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("probe height: ", height );
    SERIAL_ECHOPAIR(", displacement: ", s_probeDisplacement);
    SERIAL_ECHOPAIR(", measurement: ", measurement);
    SERIAL_EOL;
  }
  return 0;
}

int probe(Tool tool, float& measurement, float speed, float additionalRetractDistance) {
  float fastMeasurement;
  float retractDistance = .5;
  if (
    confirmMountedAndNotTriggered("probe", tool, TOOLS_PROBE) ||

    moveToLimit(Z_AXIS, -1) ||
    s_recordMeasurement(fastMeasurement) ||
    retractFromSwitch(Z_AXIS, -1, retractDistance) ||

    moveToLimit(Z_AXIS, -1, speed, retractDistance + 0.030) ||
    s_recordMeasurement(measurement) ||
    retractToolConditionally(s_probeDisplacement, additionalRetractDistance)
  ) {
    return -1;
  }

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("fastMeasurement: ", fastMeasurement);
    SERIAL_ECHOPAIR(", slowMeasurement: ", measurement);
    SERIAL_ECHOPAIR(", delta: ", fastMeasurement - measurement);
    SERIAL_EOL;
  }

  return 0;
}

int calibrateKeyPositions(Tool tool, long cycles) {
  if (s_partiallyPrepareProbe("calibrate positions", tool)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to calibrate positions, could not prepare probe");
    return -1;
  }

  // Find the center of xy-positioner without using configured Z
  // Note: We don't use the configured Z position because we have not homed Z.
  // We don't home Z because we'd need a reliable hardcoded x,y position for
  // the z-switch, which has proven difficult. Meanwhile, the xy-position is
  // more tolerant of inaccuracies in the hardcoded values.
  float centerX;
  float centerY;
  if (xyPositionerFindCenter(tool, cycles, centerX, centerY, usePlateBackOffForZ)) {
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
  if (xyPositionerFindCenter(tool, cycles, centerX, centerY, useConfiguredZ)) {
    return -1;
  }

  // Set the x,y position of the xy-positioner
  xypos_x_pos = centerX;
  xypos_y_pos = centerY;

  // Success
  return 0;
}
