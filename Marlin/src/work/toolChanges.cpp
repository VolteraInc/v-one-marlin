#include "../../Marlin.h"
#include "../api/api.h"
#include "../vone/VOne.h"

static bool s_enabled = true;

bool getToolDetectionEnabled() { return s_enabled; }
void enableToolDetection(bool enable) { s_enabled = enable; }

static tools::Tool* s_toTool(enum ToolStates state) {
  auto& tb = vone->toolBox;
  switch (state) {
    case TOOL_STATE_PROBE_MOUNTED:
    case TOOL_STATE_TRIGGERED:
      return &tb.probe;

    case TOOL_STATE_ROUTER_MOUNTED:
      return &tb.router;

    case TOOL_STATE_NOT_MOUNTED: {
      if (tb.dispenser.attached()) {
        return &tb.dispenser;
      } else {
        return &tb.nullTool;
      }
    }

    default:
      return &tb.nullTool;
  }
}

static bool s_possibleToolChange(float voltage) {
  const auto type = classifyVoltage(voltage);
  return s_toTool(type)->detached();
}

tools::Tool* determineTool() {
  return s_toTool(determineToolState());
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
  const auto voltage = vone->pins.ptop.value().voltage;
  if (s_possibleToolChange(voltage)) {
    const auto newTool = determineTool();

    // Log a notice if we falsely detect a possible tool change
    // If we see this, it's may imply overlapping use of PTop
    // (and perhaps a subtle bug).
    // Note: Voltage will start around 0 because we hold
    //       voltage low on boot, to reset the attached tool
    //       hence the "now > 600"
    if (newTool->attached() && now > 600) {
      const auto type = classifyVoltage(voltage);
      SERIAL_ECHO_START;
      SERIAL_PAIR("NOTICE: Voltage variation detected, value:", voltage);
      SERIAL_PAIR(" type:", toolStateAsString(type));
      SERIAL_PAIR(" time:", now);
      SERIAL_EOL;
    }

    vone->toolBox.setTool(newTool);
  }
}
