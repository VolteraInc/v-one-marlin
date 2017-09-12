#include "../../Marlin.h"
#include "../../temperature.h"
#include "../api/api.h"

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
  return tool != s_toTool(tool, classifyVoltage(tool, voltage));
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
  const auto voltage = get_p_top_voltage();
  if (s_possibleToolChange(tool, voltage)) {
    const auto newTool = determineTool(tool);
    setTool(newTool);
  }
}
