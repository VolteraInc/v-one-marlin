#include "../../Marlin.h"
#include "../api/api.h"
#include "../vone/VOne.h"

static bool s_enabled = true;

bool getToolDetectionEnabled() { return s_enabled; }
void enableToolDetection(bool enable) { s_enabled = enable; }

static Tool s_toTool(Tool tool, enum ToolStates state) {
  switch (state) {
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

static bool s_possibleToolChange(Tool tool, float voltage) {
  const auto type = classifyVoltage(tool, voltage);
  return tool != s_toTool(tool, type);
}

Tool determineTool(Tool tool) {
  return s_toTool(tool, determineToolState(tool));
}

void toolChanges() {
  // Run periodically
  static unsigned long nextCheckAt = 0;
  const auto now = millis();
  if (now < nextCheckAt) {
    return;
  }
  nextCheckAt = now + 1000;

  // Note: we check this after the run-periodically code so that
  // enabling/disabling does not impact the timing of tool change
  // checks relative other tasks -- not critical, but it makes for
  // a more consistent/predictable system
  if (!getToolDetectionEnabled()) {
    return;
  }

  // Update the tool if necessary
  const auto tool = getTool();
  const auto voltage = vone->pins.ptop.value().voltage;
  if (s_possibleToolChange(tool, voltage)) {
    const auto newTool = determineTool(tool);

    // Log a notice if we falsely detect a possible tool change
    // If we see this, it's may imply overlapping use of PTop
    // (and perhaps a subtle bug).
    // Note: Voltage will start around 0 because we hold
    //       voltage low on boot, to reset the attached tool
    //       hence the "now > 600"
    if (newTool == getTool() && now > 600) {
      const auto type = classifyVoltage(tool, voltage);
      SERIAL_ECHO_START;
      SERIAL_PAIR("NOTICE: Voltage variation detected, value:", voltage);
      SERIAL_PAIR(" type:", toolStateAsString(type));
      SERIAL_PAIR(" time:", now);
      SERIAL_EOL;
    }

    setTool(newTool);
  }
}
