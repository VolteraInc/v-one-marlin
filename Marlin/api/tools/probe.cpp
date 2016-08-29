#include "../../Marlin.h"
#include "../../stepper.h"
#include "../api.h"
#include "internal.h"

static float s_probeDisplacement = 0.0f;

#if VOLTERA_PIN_VERSION == 1 // For B1 units?

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

float readStableProbePinVoltage() {
  int maxIterations = 100; // Number of times it'll try to read a stable voltage before giving up
  int count = 0;
  int reportThreshold = 8;
  float referenceVoltage = readProbePinVoltage(); // Reference to check against

  for(int i = 0; i < maxIterations; i++) {
    delay(10);
    float voltage = readProbePinVoltage();

    if(voltage != referenceVoltage) {
      referenceVoltage = voltage; // updates, smart!
      count = 0;  // reset counter if subsequent states differ.

    } else if(++count >= 4) {
      if (i+1 >= reportThreshold) { // to report that we were coming close to failing
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Warning: determine stable probe pin voltage took "); SERIAL_ECHO(i+1);
        SERIAL_ECHOPGM(" of "); SERIAL_ECHO(maxIterations);
        SERIAL_ECHOLNPGM(" iterations to resolve.");
      }

      if(logging_enabled) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Determined stable probe pin voltage "); SERIAL_ECHO(voltage);
        SERIAL_ECHOPGM(" after "); SERIAL_ECHO(i+1);
        SERIAL_ERRORLNPGM(" iterations.");
      }
      return voltage;
    }
  }
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Warning: Failed to determine a stable probe pin voltage, too much variation in readings.");
  return -1.0; // If we fail

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

int probe(Tool tool, float& measurement) {
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
  return 0;
}
