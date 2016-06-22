#include "../../Marlin.h"

#include "../api.h"
#include "../internal.h"

#include "internal.h"

static Tool s_tool = TOOLS_NONE;

static bool s_probeReady = false;
static bool s_dispenserReady = false;

void sendToolStatusUpdate() {
  SERIAL_PROTOCOLPGM("toolUpdate");
  SERIAL_PROTOCOLPGM(" type:"); SERIAL_PROTOCOL(toolTypeAsString(s_tool));
  SERIAL_PROTOCOLPGM("\n");
}

int prepareToolToMove(Tool tool) {
  // Ensure homed in X, Y
  if (!homedXY()) {
    if (homeXY()) {
      return -1;
    }
  }

  // Perform tool-specific preparations
  switch (tool) {
    case TOOLS_PROBE:
      if (!s_probeReady) {
        if (prepareProbe(tool)) {
          return -1;
        }
        s_probeReady = true;
      }
      break;

    case TOOLS_DISPENSER:
      if (!s_dispenserReady) {
        if (prepareDispenser(tool)) {
          return -1;
        }
        s_dispenserReady = true;
      }
      break;

    case TOOLS_NONE:
      break;
  }
  return 0;
}

void setTool(Tool tool) {
  if (logging_enabled) {
    SERIAL_ECHOPGM("Swapping "); SERIAL_ECHO(toolTypeAsString(s_tool));
    SERIAL_ECHOPGM(" for "); SERIAL_ECHO(toolTypeAsString(tool));
    SERIAL_ECHOPGM("\n");
  }
  if (s_tool == tool) {
    return;
  }
  s_tool = tool;
  resetToolPreparations();
  sendToolStatusUpdate();
}

Tool getTool() {
  return s_tool;
}

int resetToolPreparations() {
  switch (s_tool) {
    case TOOLS_PROBE:
      s_probeReady = false;
      setDispenseHeight(TOOLS_DISPENSER, 0.0f);
      s_dispenserReady = false;
      setHomedState(Z_AXIS, 0);
      break;
    case TOOLS_DISPENSER:
      setDispenseHeight(TOOLS_DISPENSER, 0.0f);
      s_dispenserReady = false;
      setHomedState(Z_AXIS, 0);
      break;
    case TOOLS_NONE:
      break;
  }
  return 0;
}

const char* toolTypeAsString(Tool tool) {
  switch(tool) {
    case TOOLS_NONE: return "None";
    case TOOLS_PROBE: return "Probe";
    case TOOLS_DISPENSER: return "Dispenser";
    default: return "unknown";
  }
}

int outputToolStatus() {
  SERIAL_ECHOPGM("Tool");
  SERIAL_ECHOPGM(" type:"); SERIAL_ECHOLN(toolTypeAsString(s_tool));
  SERIAL_ECHOPGM("\n");

  SERIAL_ECHOPGM("Probe ");
  SERIAL_ECHOPGM(" trigger status: "); SERIAL_ECHOLN(probeTriggerStateAsString(readProbeTriggerState()));
  SERIAL_ECHOPGM("\n");

  SERIAL_ECHOPGM("Dispenser ");
  SERIAL_ECHOPGM(" dispense height: "); SERIAL_ECHOLN(getDispenseHeight(TOOLS_DISPENSER));
  SERIAL_ECHOPGM("\n");

  SERIAL_ECHOPGM("Homing");
  SERIAL_ECHOPGM(" x:"); SERIAL_ECHOLN(getHomedState(X_AXIS));
  SERIAL_ECHOPGM(" y:"); SERIAL_ECHOLN(getHomedState(Y_AXIS));
  SERIAL_ECHOPGM(" z:"); SERIAL_ECHOLN(getHomedState(Z_AXIS));
  SERIAL_ECHOPGM("\n");

  SERIAL_ECHOPGM("Status");
  SERIAL_ECHOPGM(" Probe Ready:"); SERIAL_ECHOLN(s_probeReady);
  SERIAL_ECHOPGM(" Dispenser Ready:"); SERIAL_ECHOLN(s_dispenserReady);
  SERIAL_ECHOPGM("\n");

  return 0;
}

int move(Tool tool, float x, float y, float z, float e, float f, bool applyDispenseHeight) {
  if (applyDispenseHeight) {
    z += getDispenseHeight(tool);
  }

  return rawMove(x, y, z, e, f);
}

int moveXY(Tool tool, float x, float y, float f) {
  return move(tool, x, y, current_position[Z_AXIS], current_position[E_AXIS], f);
}

int moveZ(Tool tool, float z, float f, bool applyDispenseHeight) {
  return move(tool, current_position[X_AXIS], current_position[Y_AXIS], z, current_position[E_AXIS], f, applyDispenseHeight);
}

int relativeMove(Tool tool, float x, float y, float z, float e, float speed_in_mm_per_min) {
  return move(
    tool,
    current_position[ X_AXIS ] + x,
    current_position[ Y_AXIS ] + y,
    current_position[ Z_AXIS ] + z,
    current_position[ E_AXIS ] + e,
    speed_in_mm_per_min
  );
}
