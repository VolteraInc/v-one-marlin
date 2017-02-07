#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"

static float s_probeDisplacement = 0.0f;

bool probeMounted() {
  switch (readProbeTriggerState()) {
    case PROBE_TRIGGERED:
    case PROBE_ON: return true;

    case PROBE_OFF:
    case PROBE_UNKNOWN: return false;
  }
  return false;
}

int prepareProbe(Tool tool) {
  if (raise()) {
    return -1;
  }

  // Check for errors
  switch (readProbeTriggerState()) {
    case PROBE_ON:
      break;

    case PROBE_OFF:
    case PROBE_UNKNOWN:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to prepare probe, probe not mounted");
      return -1;

    case PROBE_TRIGGERED:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to prepare probe, probe detected contact in raised position");
      return -1;
  }

  // Ensure homed in Z
  if (!homedZ()) {
    if (homeZ(tool)) {
     return -1;
    }
  }

  Point2D toolPosition;
  if (xyPositionerFindCenter(tool, defaultXyPositionerCycles, toolPosition.x, toolPosition.y)) {
    return -1;
  }

  auto const dx = abs(xypos_x_pos - toolPosition.x);
  auto const dy = abs(xypos_y_pos - toolPosition.y);

  if(dx > 1 || dy > 1){
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Calculated XY center is very different than calibrated. ");
    SERIAL_ERRORPGM("Calibrated X: "); SERIAL_ERROR(xypos_x_pos);
    SERIAL_ERRORPGM(" Y: "); SERIAL_ERROR(xypos_y_pos);
    SERIAL_ERRORPGM(" Measured X: "); SERIAL_ERROR(toolPosition.x);
    SERIAL_ERRORPGM(" Y: "); SERIAL_ERRORLN(toolPosition.y);
    return -1;
  }

  // Overwrite the current position with constant position.
  setPosition(xypos_x_pos, xypos_y_pos, current_position[Z_AXIS], current_position[E_AXIS]);

  // Record the probe's displacement
  if (measureProbeDisplacement(tool, s_probeDisplacement)) {
    return -1;
  }

  if (raise()) {
    return -1;
  }

  return 0;
}

int probe(Tool tool, float& measurement, float additionalRetractDistance) {
  if(tool != TOOLS_PROBE) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to probe, probe not attached");
    return -1;
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Check for errors
  switch (readProbeTriggerState()) {
    case PROBE_ON:
      break;

    case PROBE_OFF:
    case PROBE_UNKNOWN:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to probe, probe not mounted");
      return -1;

    case PROBE_TRIGGERED:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to probe, probe detected contact before movement started");
      return -1;
  }

  // Move to limit
  if (moveToLimit(Z_AXIS, -1) != 0) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to probe, switch did not trigger");
    return -1;
  }

  // Record the measurement
  measurement = current_position[Z_AXIS] + s_probeDisplacement;
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("probe height: "); SERIAL_ECHOLN(current_position[Z_AXIS]);
    SERIAL_ECHOPGM("probe displacement: "); SERIAL_ECHOLN(s_probeDisplacement);
    SERIAL_ECHOPGM("probe measurement: "); SERIAL_ECHOLN(measurement);
  }

  // Retract, conditionally
  if (additionalRetractDistance != NoRetract) {
    if (retractFromSwitch(Z_AXIS, -1, s_probeDisplacement + additionalRetractDistance)) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to probe, retract did not complete");
      return -1;
    }
  }

  return 0;
}
