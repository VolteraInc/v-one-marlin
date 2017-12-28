#include "../../../Marlin.h"
#include "../../../stepper.h"

#include "../../api/api.h"


void Tool::activate() {
  m_active = true;
}

void Tool::deactivate() {
  m_active = false;
  _unprepare();
}

static bool s_toolPrepared = false;

static int s_prepareTool(Tool tool) {
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Preparing tool");
  switch (tool) {
    case TOOLS_PROBE: return Probe::prepare(tool);
    case TOOLS_DISPENSER: return Dispenser::prepare(tool);
    case TOOLS_ROUTER: return Router::prepare(tool);
    default: return ensureHomedInXY();
  }
}

int prepareToolToMove(Tool tool) {
  if (!s_toolPrepared) {
    if (s_prepareTool(tool)) {
      return -1;
    }
    s_toolPrepared = true;
  }
  return 0;
}

static int s_unprepareTool(Tool tool) {
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Reset tool preparations");
  switch (tool) {
    case TOOLS_PROBE: return Probe::unprepare(tool);
    case TOOLS_DISPENSER: return Dispenser::unprepare(tool);
    case TOOLS_ROUTER: return Router::unprepare(tool);
    case TOOLS_NONE: return NoTool::unprepare(tool);
    default: return 0;
  }
}

int resetToolPreparations() {
  if (s_toolPrepared) {
    auto tool = getTool();
    if (s_unprepareTool(tool)) {
      return -1;
    }
    s_toolPrepared = false;
  }
  return 0;
}
