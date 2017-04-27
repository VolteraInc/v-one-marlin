#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"

static float s_probeDisplacement = 0.0f;

int prepareProbe(Tool tool) {
  const char* context = "prepare probe";
  enable_p_top(true);
  return (
    raise() ||
    confirmMountedAndNotTriggered(context, tool, TOOLS_PROBE) ||
    ensureHomedInXY() ||
    ensureHomedInZ(tool) ||
    centerTool(tool) ||
    measureProbeDisplacement(tool, s_probeDisplacement) ||
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

int probe(Tool tool, float& measurement, float additionalRetractDistance) {
  return (
    confirmMountedAndNotTriggered("probe", tool, TOOLS_PROBE) ||
    lowerUntilToolContacts(tool) ||
    s_recordMeasurement(measurement) ||
    retractToolConditionally(s_probeDisplacement, additionalRetractDistance)
  );
}
