#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "../../vone/VOne.h"

enum ToolStates classifyVoltage(Tool, float voltage) {
  // if (logging_enabled) {
  //   SERIAL_ECHO_START;
  //   SERIAL_ECHOPGM("Classifying voltage "); SERIAL_ECHOLN(voltage);
  // }
  if (voltage < 0.08) { // expected 0.07 (once stable)
    return TOOL_STATE_TRIGGERED;
  } else if (voltage <= 3.2) {
    // Note: ROUTER gives 2.06 when mounted and not powered
    // we don't treat this as a unique state right now, but
    // we don't want to clarify this as a PROBE.
    return TOOL_STATE_NOT_MOUNTED;
  } else if (voltage <= 3.8) { // expecting 3.55 (once stable)
    return TOOL_STATE_PROBE_MOUNTED;
  } else if (voltage <= 4.5) { // expecting 4.22 (once stable)
    return TOOL_STATE_ROUTER_MOUNTED;
  } else {
    return TOOL_STATE_NOT_MOUNTED;
  }
}

enum ToolStates determineToolState(Tool tool) {
  // Stabilize the reading
  // Notes:
  //    - There is a capacitor on the p_top line, this means the voltage
  //      falls quickly, but climbs slowly. e.g climbing from 3.5 to 5 take about 30ms.
  //    - We use a counter to detect consecutive equivalent classifications because
  //      averaging would add even more lag.
  int maxIterations = 10;
  int warningThreshold = 4;
  int count = 0;
  static enum ToolStates previousState = TOOL_STATE_UNKNOWN;
  const auto start = millis();
  for (int i = 0; i < maxIterations; ++i) {
    enum ToolStates state = classifyVoltage(tool, vone->pins.ptop.readValue().voltage);

    if (previousState == state) {
      ++count;

      // Return state if it has not changed since this function was last called
      // or if we've collect enough consistent consecutive readings
      if (i == 0 || count >= 2) { // TODO: need to retest this...will likely need to increase or add a delay
        if (i + 1 >= warningThreshold) {
          SERIAL_ECHO_START;
          SERIAL_ECHOPGM("Warning: determination of tool state took "); SERIAL_ECHO(i+1);
          SERIAL_ECHOPGM(" of "); SERIAL_ECHO(maxIterations);
          SERIAL_ECHOLNPGM(" iterations to resolve.");
        }

        const auto stop = millis();
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Determined tool state '"); SERIAL_ECHO(toolStateAsString(state));
        SERIAL_ECHOPGM("' after "); SERIAL_ECHO(i+1);
        SERIAL_ECHOPGM(" iterations ("); SERIAL_ECHO(stop - start);
        SERIAL_ECHOLNPGM(" ms)");

        return state;
      }
    } else {
      // classification differs from previous, reset counter
      count = 1;
      previousState = state;
    }
  }

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Warning: Unable to determine tool's state, too much variation in readings.");
  return TOOL_STATE_UNKNOWN;
}

const char* toolStateAsString(enum ToolStates state) {
  switch(state) {
    case TOOL_STATE_TRIGGERED: return "Triggered";
    case TOOL_STATE_PROBE_MOUNTED: return "Probe Mounted";
    case TOOL_STATE_ROUTER_MOUNTED: return "Router Mounted";
    case TOOL_STATE_NOT_MOUNTED: return "Not Mounted";
    case TOOL_STATE_UNKNOWN: return "Unknown";
  }
  return "Unknown";
}
