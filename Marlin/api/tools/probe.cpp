#include "../../Marlin.h"
#include "../../stepper.h"
#include "../api.h"
#include "internal.h"

static float s_probeDisplacement = 0.0f;

#if VOLTERA_PIN_VERSION == 1

float readProbePinVoltage() {
  return 5.0;
}

enum ProbeTriggerStates readProbeTriggerState() {
  return getTool() == TOOLS_PROBE ? PROBE_ON : PROBE_OFF;
}

#else

float readProbePinVoltage() {
  return analogRead(P_TOP_STATE_PIN) / 1024.0 * 5.0;
}

static enum ProbeTriggerStates s_classifyVoltage(float voltage) {
  // Determine state
  if (voltage < 1.0) {
    return PROBE_TRIGGERED;
  } else if (voltage <= 4.0) {
    return PROBE_ON;
  } else {
    return PROBE_OFF;
  }
}

enum ProbeTriggerStates readProbeTriggerState() {
  // Stabilize the reading
  // Note: We've seen an occasional reading of 5.0 in otherwise stable readings,
  // if the readings are ~1.0 than a simple average could take too many iterations to
  // reach <1.0. Hence, we use a counter to detect consecutive equivalent classifications.
  int maxIterations = 10;
  int reportThreshold = 8;
  int count = 0;
  enum ProbeTriggerStates state = PROBE_OFF;
  enum ProbeTriggerStates previousState = s_classifyVoltage(readProbePinVoltage());
  for (int i = 0; i < maxIterations; ++i) {
    delay(1);
    state = s_classifyVoltage(readProbePinVoltage());

    // Reset counter if state differs from previous reading
    if (previousState != state) {
      previousState = state;
      count = 0;
      continue;
    }

    // Return state when we've collect enough consistent consecutive readings
    if (++count >= 4) {
      if (i + 1 >= reportThreshold) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Warning: determine probe trigger state took "); SERIAL_ECHO(i+1);
        SERIAL_ECHOPGM(" of "); SERIAL_ECHO(maxIterations);
        SERIAL_ECHOLNPGM(" iterations to resolve.");
      }
      if (logging_enabled) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Determined probe trigger state "); SERIAL_ECHO(probeTriggerStateAsString(state));
        SERIAL_ECHOPGM(" after "); SERIAL_ECHO(i+1);
        SERIAL_ECHOLNPGM(" iterations");
      }
      return state;
    }
  }

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Warning: Unable to determine probe trigger state, too much variation in readings.");
  return PROBE_UNKNOWN;
}

#endif


const char* probeTriggerStateAsString(enum ProbeTriggerStates state) {
  switch(state) {
    case PROBE_TRIGGERED: return "TRIGGERED";
    case PROBE_ON: return "ON";
    case PROBE_OFF: return "OFF";
    case PROBE_UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

static bool s_probeMounted() {
  switch (readProbeTriggerState()) {
    case PROBE_TRIGGERED:
    case PROBE_ON: return true;

    case PROBE_OFF:
    case PROBE_UNKNOWN: return false;
  }
  return false;
}

int prepareProbe(Tool tool, Point2D& reference) {
  if(!s_probeMounted()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to prepare probe, probe not mounted");
    return -1;
  }

  // Ensure homed in Z
  if (!homedZ()) {
    if (homeZ(tool)) {
     return -1;
    }
  }

  // Record the center of the xy-positioner for use as a references
  if (xyPositionerFindCenter(tool, defaultXyPositionerCycles, reference.x, reference.y)) {
    return -1;
  }

  // Record the probe's displacement
  if (measureProbeDisplacement(tool, s_probeDisplacement)) {
    return -1;
  }

  if (raise()) {
    return -1;
  }

  return 0;
}

int probe(Tool tool, float& measurement) {
  if(tool != TOOLS_PROBE) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to probe, probe not attached");
    return -1;
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  if(!s_probeMounted()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to probe, probe not mounted");
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
  return 0;
}
