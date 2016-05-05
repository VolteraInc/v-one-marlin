#include "../../Marlin.h"
#include "../../stepper.h"
#include "../api.h"
#include "internal.h"

static float s_probeDisplacement = 0.0f;

static bool s_probeMounted() {
#if VOLTERA_PIN_VERSION == 1
  return getTool() == TOOLS_PROBE;
#else
  float probeVoltage = analogRead(P_TOP_STATE_PIN) / 1024.0 * 5.0;
  return probeVoltage <= 4.0;
#endif
}

int prepareProbe(Point2D& reference) {

  if (!s_probeMounted()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to probe, probe not mounted");
    return -1;
  }

  // Ensure homed in Z
  if (!homedZ()) {
    if (homeZ()) {
     return -1;
    }
  }

  // Record the center of the xy-positioner for use as a references
  if (xyPositionerFindCenter(defaultXyPositionerCycles, reference.x, reference.y)) {
    return -1;
  }

  // Record the probe's displacement
  if (measureProbeDisplacement(s_probeDisplacement)) {
    return -1;
  }

  if (raise()) {
    return -1;
  }

  return 0;
}

int probe(float& measurement) {
  // Finish any pending moves (prevents crashes)
  st_synchronize();

  if (!s_probeMounted()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to probe, probe not mounted");
    return -1;
  }

  // Move to limit
  if (moveToLimit(Z_AXIS, -1) != 0) {
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to probe, switch did not trigger\n");
    return -1;
  }

  // Record the measurement
  measurement = current_position[Z_AXIS] + s_probeDisplacement;
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHO("probe height: "); SERIAL_ECHOLN(current_position[Z_AXIS]);
    SERIAL_ECHO("probe displacement: "); SERIAL_ECHOLN(s_probeDisplacement);
    SERIAL_ECHO("probe measurement: "); SERIAL_ECHOLN(measurement);
  }
  return 0;
}
