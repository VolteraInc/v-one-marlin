#include "../../../Marlin.h"
#include "../../../temperature.h" // get_p_top_voltage
#include "../../../stepper.h"
#include "../api.h"

static enum ToolStates s_classifyVoltage(Tool, float voltage) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Classifying voltage "); SERIAL_ECHOLN(voltage);
  }
  if (voltage < 1.0) { // expected 0.7 (once stable)
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

enum ToolStates getToolState(Tool tool) {
  return s_classifyVoltage(tool, get_p_top_voltage());
}

Tool determineMountedTool(Tool tool) {
  switch (getToolState(tool)) {
    case TOOL_STATE_PROBE_MOUNTED:
    case TOOL_STATE_TRIGGERED:
      return TOOLS_PROBE;

    case TOOL_STATE_ROUTER_MOUNTED:
      return TOOLS_ROUTER;

    case TOOL_STATE_NOT_MOUNTED:
      return tool == TOOLS_DISPENSER ? TOOLS_DISPENSER : TOOLS_NONE;

    default:
      return TOOLS_NONE;
  }
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
