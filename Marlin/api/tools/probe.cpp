#include "../../Marlin.h"
#include "../../stepper.h"
#include "../api.h"
#include "internal.h"

static float s_probeDisplacement = 0.0f;

float readProbePinVoltage() {
#if defined(P_TOP_STATE_PIN) && P_TOP_STATE_PIN > -1
  return analogRead(P_TOP_STATE_PIN) / 1024.0 * 5.0;
#else
  return 5.0;
#endif
}

static enum ProbeTriggerStates classifyVoltage(float voltage) {
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
#if VOLTERA_PIN_VERSION == 1
  return getTool() == TOOLS_PROBE ? PROBE_ON : PROBE_OFF;
#else
  // Stabilize the reading
  // Note: We've seen an occasional reading of 5.0 in otherwise stable readings,
  // if the readings are ~1.0 than a simple average could take too many iterations to
  // reach <1.0. Hence, we use a counter to detect consecutive equivalent classifications.
  int maxIterations = 10;
  int reportThreshold = 8;
  int count = 0;
  enum ProbeTriggerStates state = PROBE_OFF;
  enum ProbeTriggerStates previousState = classifyVoltage(readProbePinVoltage());
  for (int i = 0; i < maxIterations; ++i) {
    delay(1);
    state = classifyVoltage(readProbePinVoltage());

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
        SERIAL_ECHO("Warning: determine probe trigger state took "); SERIAL_ECHO(i+1);
        SERIAL_ECHO(" of "); SERIAL_ECHO(maxIterations);
        SERIAL_ECHOLN(" iterations to resolve.");
      }
      if (logging_enabled) {
        SERIAL_ECHO_START;
        SERIAL_ECHO("Determined probe trigger state "); SERIAL_ECHO(probeTriggerStateAsString(state));
        SERIAL_ECHO(" after "); SERIAL_ECHO(i+1);
        SERIAL_ECHOLN(" iterations");
      }
      return state;
    }
  }

  SERIAL_ECHO_START;
  SERIAL_ECHO("Warning: Unable to determine probe trigger state, too much variation in readings.");
  return PROBE_UNKNOWN;
#endif
}

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

int prepareProbe(Point2D& reference) {
  if(!s_probeMounted()) {
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

  if(!s_probeMounted()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to probe, probe not mounted");
    return -1;
  }


  // Move to limit
  if (moveToLimit(Z_AXIS, -1) != 0) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to probe, switch did not trigger");
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
