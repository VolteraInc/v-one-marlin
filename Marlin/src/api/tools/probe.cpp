#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"

extern float axis_steps_per_unit[4];
static float s_probeDisplacement = 0.0f;

int partiallyPrepareProbe(const char* context, Tool tool) {
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
    partiallyPrepareProbe(context, tool) ||
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
