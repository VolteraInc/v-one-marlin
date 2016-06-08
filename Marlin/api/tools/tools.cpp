#include "../../Marlin.h"
#include "../api.h"
#include "internal.h"


static Tool s_tool = TOOLS_NONE;
static Point2D s_reference;

static bool s_probeReady = false;
static bool s_dispenserReady = false;

void sendToolStatusUpdate() {
  SERIAL_PROTOCOLPGM("toolUpdate");
  SERIAL_PROTOCOLPGM(" type:"); SERIAL_PROTOCOL(toolTypeAsString(s_tool));
  SERIAL_PROTOCOLPGM("\n");
}

int prepareToolToMove() {
  // Ensure homed in X, Y
  if (!homedXY()) {
    if (homeXY()) {
      return -1;
    }
  }

  // Perform tool-specific preparations
  switch (s_tool) {
    case TOOLS_PROBE:
      if (!s_probeReady) {
        if (prepareProbe(s_reference)) {
          return -1;
        }
        s_probeReady = true;
      }
      break;

    case TOOLS_DISPENSER:
      if (!s_dispenserReady) {
        if (prepareDispenser(s_reference)) {
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
      s_dispenserReady = false;
      setHomedState(Z_AXIS, 0);
      break;
    case TOOLS_DISPENSER:
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
  SERIAL_ECHOPGM(" type:"); SERIAL_ECHO(toolTypeAsString(s_tool));
  SERIAL_ECHOPGM("\n");

  SERIAL_ECHOPGM("Probe ");
  SERIAL_ECHOPGM(" trigger status: "); SERIAL_ECHO(probeTriggerStateAsString(readProbeTriggerState()));
  SERIAL_ECHOPGM("\n");

  SERIAL_ECHOPGM("Homing");
  SERIAL_ECHOPGM(" x:"); SERIAL_ECHO(getHomedState(X_AXIS));
  SERIAL_ECHOPGM(" y:"); SERIAL_ECHO(getHomedState(Y_AXIS));
  SERIAL_ECHOPGM(" z:"); SERIAL_ECHO(getHomedState(Z_AXIS));
  SERIAL_ECHOPGM("\n");

  SERIAL_ECHOPGM("Reference ");
  SERIAL_ECHOPGM(" Probe Ready:"); SERIAL_ECHO(s_probeReady);
  SERIAL_ECHOPGM(" Dispenser Ready:"); SERIAL_ECHO(s_dispenserReady);
  SERIAL_ECHOPGM(" reference.x:"); SERIAL_ECHO(s_reference.x);
  SERIAL_ECHOPGM(" reference.y:"); SERIAL_ECHO(s_reference.y);
  SERIAL_ECHOPGM("\n");

  return 0;
}
