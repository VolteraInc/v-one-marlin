#include "../../../Marlin.h"
#include "../../../planner.h"
#include "../api.h"
#include "internal.h"

static float s_dispenseHeight = 0.0f;

int prepareDispenser(Tool tool) {
  const char* context = "prepare dispenser";
  return (
    raise() ||
    meshGears() ||
    confirmRequiredToolAttached(context, tool, TOOLS_DISPENSER) ||
    confirmMountedAndNotTriggered(context, tool) ||
    ensureHomedInXY() ||
    ensureHomedInZ(tool) ||
    centerTool(tool) ||
    raise()
  );
}

float getDispenseHeight(Tool tool) {
  // Confirm we have a dispenser
  if (tool != TOOLS_DISPENSER) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Warning: dispense height requested for "); SERIAL_ERROR(toolTypeAsString(tool));
    SERIAL_ECHOPGM(" returning "); SERIAL_ECHOLN(s_dispenseHeight);
  }
  return s_dispenseHeight;
}

int setDispenseHeight(Tool tool, float height) {
  if (logging_enabled) {
    SERIAL_ECHOPGM("Dispense height set to "); SERIAL_ECHO(height);
    SERIAL_ECHOPGM("mm\n");
  }

  // Confirm we have a dispenser
  if (tool != TOOLS_DISPENSER) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set dispensing height, tool is "); SERIAL_ERRORLN(toolTypeAsString(tool));
    return -1;
  }

  if (height < 0.0f || height > 2.0f ) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set dispensing height to "); SERIAL_ERROR(height);
    SERIAL_ERRORPGM("mm, value is outside expected range\n");
    return -1;
  }

  // Set the height
  s_dispenseHeight = height;

  return 0;
}
