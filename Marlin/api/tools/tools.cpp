#include "../../Marlin.h"
#include "../api.h"
#include "internal.h"


static Tool s_tool = TOOLS_NONE;
static Point2D s_reference;

static bool s_probeReady = false;
static bool s_dispenserReady = false;

void sendToolStatusUpdate() {
  SERIAL_PROTOCOL("toolUpdate");
  SERIAL_PROTOCOL(" type:"); SERIAL_PROTOCOL(toolTypeAsString(s_tool));
  SERIAL_PROTOCOL("\n");
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
    SERIAL_ECHO("Swapping ");
    SERIAL_ECHO(toolTypeAsString(s_tool));
    SERIAL_ECHO(" for ");
    SERIAL_ECHO(toolTypeAsString(tool));
    SERIAL_ECHO("\n");
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
  SERIAL_ECHO("Tool");
  SERIAL_ECHO(" type:"); SERIAL_ECHO(toolTypeAsString(s_tool));
  SERIAL_ECHO("\n");

  SERIAL_ECHO("Probe ");
  SERIAL_ECHO(" trigger status: "); SERIAL_ECHO(probeTriggerStateAsString(readProbeTriggerState()));
  SERIAL_ECHO("\n");

  SERIAL_ECHO("Homing");
  SERIAL_ECHO(" x:"); SERIAL_ECHO(getHomedState(X_AXIS));
  SERIAL_ECHO(" y:"); SERIAL_ECHO(getHomedState(Y_AXIS));
  SERIAL_ECHO(" z:"); SERIAL_ECHO(getHomedState(Z_AXIS));
  SERIAL_ECHO("\n");

  SERIAL_ECHO("Reference ");
  SERIAL_ECHO(" Probe Ready:"); SERIAL_ECHO(s_probeReady);
  SERIAL_ECHO(" Dispenser Ready:"); SERIAL_ECHO(s_dispenserReady);
  SERIAL_ECHO(" reference.x:"); SERIAL_ECHO(s_reference.x);
  SERIAL_ECHO(" reference.y:"); SERIAL_ECHO(s_reference.y);
  SERIAL_ECHO("\n");

  return 0;
}
