#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"

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
  measurement = current_position[Z_AXIS] + s_probeDisplacement;
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("probe height: "); SERIAL_ECHOLN(current_position[Z_AXIS]);
    SERIAL_ECHOPGM("probe displacement: "); SERIAL_ECHOLN(s_probeDisplacement);
    SERIAL_ECHOPGM("probe measurement: "); SERIAL_ECHOLN(measurement);
  }
  return 0;
}

int probe(Tool tool, float& measurement, float speed, float additionalRetractDistance) {
  return (
    confirmMountedAndNotTriggered("probe", tool, TOOLS_PROBE) ||
    moveToLimit(Z_AXIS, -1, speed) ||
    s_recordMeasurement(measurement) ||
    retractToolConditionally(s_probeDisplacement, additionalRetractDistance)
  );
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
