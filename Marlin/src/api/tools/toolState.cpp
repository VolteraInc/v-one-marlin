#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"

float readPogoPinVoltage() {
  return analogRead(P_TOP_STATE_PIN) / 1024.0 * 5.0;
}

static enum ToolStates s_classifyVoltage(Tool tool, float voltage) {
  // The pick-and-place tool's voltage classification is inverted relative to the probe
  // TODO: see how drill behaves
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Classifying voltage "); SERIAL_ECHOLN(voltage);
  }
  const auto invert = tool == TOOLS_PNP;
  if (voltage < 1.0) {
    return invert ? TOOL_STATE_MOUNTED : TOOL_STATE_TRIGGERED;
  } else if (voltage <= 4.0) {
    return invert ? TOOL_STATE_TRIGGERED : TOOL_STATE_MOUNTED;
  } else {
    return TOOL_STATE_NOT_MOUNTED;
  }
}

enum ToolStates readToolState(Tool tool) {
  // Stabilize the reading
  // Note: We've seen an occasional reading of 5.0 in otherwise stable readings,
  // if the readings are ~1.0 than a simple average could take too many iterations to
  // reach <1.0. Hence, we use a counter to detect consecutive equivalent classifications.
  int maxIterations = 12;
  int reportThreshold = 10;
  int count = 0;
  enum ToolStates state = TOOL_STATE_UNKNOWN;
  enum ToolStates previousState = s_classifyVoltage(tool, readPogoPinVoltage());
  for (int i = 0; i < maxIterations; ++i) {
    delay(1);
    state = s_classifyVoltage(tool, readPogoPinVoltage());

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
        SERIAL_ECHOPGM("Warning: determination of probe trigger state took "); SERIAL_ECHO(i+1);
        SERIAL_ECHOPGM(" of "); SERIAL_ECHO(maxIterations);
        SERIAL_ECHOLNPGM(" iterations to resolve.");
      }
      if (logging_enabled) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Determined probe trigger state '"); SERIAL_ECHO(toolStateAsString(state));
        SERIAL_ECHOPGM("' after "); SERIAL_ECHO(i+1);
        SERIAL_ECHOLNPGM(" iterations");
      }
      return state;
    }
  }

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Warning: Unable to determine tool's state, too much variation in readings.");
  return TOOL_STATE_UNKNOWN;
}

const char* toolStateAsString(enum ToolStates state) {
  switch(state) {
    case TOOL_STATE_TRIGGERED: return "triggered";
    case TOOL_STATE_MOUNTED: return "Mounted";
    case TOOL_STATE_NOT_MOUNTED: return "not mounted";
    case TOOL_STATE_UNKNOWN: return "unknown";
  }
  return "unknown";
}
