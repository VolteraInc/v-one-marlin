#include "../../../Marlin.h"
#include "../../../planner.h"
#include "../api.h"
#include "internal.h"

static float s_dispenseHeight = 0.0f;

int meshGears() {
  // Move E backward, then move forward.
  return (
    relativeRawMoveE(-0.02) ||
    relativeRawMoveE(0.02)
  );
}

int Dispenser::prepare(Tool tool) {
  const char* context = "prepare dispenser";
  return (
    raise() ||
    confirmMountedAndNotTriggered(context, tool, TOOLS_DISPENSER) ||
    meshGears() ||

    ensureHomedInXY() ||
    homeZ(tool) || // home Z so we can enter the xy pos with decent precision
    centerTool(tool) ||
    homeZ(tool) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)

    raise()
  );
}

int Dispenser::unprepare(Tool tool) {
  setHomedState(Z_AXIS, 0);
  return (
    setDispenseHeight(tool, 0.0f)
  );
}

float Dispenser::getDispenseHeight(Tool tool) {
  // Confirm we have a dispenser
  if (tool != TOOLS_DISPENSER) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Warning: dispense height requested for "); SERIAL_ERROR(toolTypeAsString(tool));
    SERIAL_ECHOPGM(" returning "); SERIAL_ECHOLN(s_dispenseHeight);
  }
  return s_dispenseHeight;
}

int Dispenser::setDispenseHeight(Tool tool, float height) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
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
