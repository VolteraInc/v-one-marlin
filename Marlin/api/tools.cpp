#include "api.h"

#include "../Marlin.h"
#include "../planner.h"

struct Point2D {
  float x = 0.0f;
  float y = 0.0f;
};

static Tool s_tool = TOOLS_NONE;
static bool s_measuredReference = false;
static bool s_probeCalibrated = false;
static bool s_aligned = false;
static Point2D s_reference;
static float s_probeDisplacement = 0.0f;

static int s_prepareProbe() {
  bool needToRaise = false;
  int returnValue = -1;

  // Record the center of the xy-positioner for use as a references
  if (!s_measuredReference) {
    needToRaise = true;
    if (xyPositionerFindCenter(defaultXyPositionerCycles, s_reference.x, s_reference.y)) {
      goto DONE;
    }
    s_measuredReference = true;
  }

  // Record the probe's displacement
  if (!s_probeCalibrated) {
    needToRaise = true;
    if (measureProbeDisplacement(s_probeDisplacement)) {
      goto DONE;
    }
    s_probeCalibrated = true;
  }

  returnValue = 0;
DONE:
  if (needToRaise) {
    // Raise
    // Note: We are below the normal range of z-values so a normal raise() will not
    // travel far enough. We could modify raise(), but that would add complexity
    // and/or increase the risk of crashing for this one special case.
    if (current_position[Z_AXIS] < 0) {
      if (moveZ(0)) {
        return -1;
      }
    }
    if (raise()) {
      return -1;
    }
  }

  return returnValue;
}

// Camera code handed-off
// Reduced need to show Jesus code as i go
// v-code commands written, testing now,
// next task debug dispensing


static int s_alignToReference(Point2D& reference) {
  // Move the current tool to the reference position
  // i.e. the center of the xy-positioner.
  Point2D toolPosition;
  if (xyPositionerFindCenter(defaultXyPositionerCycles, toolPosition.x, toolPosition.y)) {
    return -1;
  }

  // The tip of the tool is currently at the center of the xy-positioner
  // Override the current position
  current_position[X_AXIS] = reference.x;
  current_position[Y_AXIS] = reference.y;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  return 0;
}

static int s_prepareDispenser() {
  // Measure the center of the xy-positioner to compensate for the dispenser's offset
  if (!s_aligned) {
    if (s_alignToReference(s_reference)) {
      return -1;
    }
    s_aligned = true;
  }
  return 0;
}

int prepareToolToMove() {
  // Ensure homed in X, Y
  if (!homedXY()) {
    if (homeXY()) {
      return -1;
    }
  }

  // Ensure homed in Z
  if (s_tool != TOOLS_NONE && !homedZ()) {
    if (homeZ()) {
     return -1;
    }
  }

  // Perform tool-specific preparations
  switch (s_tool) {
    case TOOLS_PROBE: return s_prepareProbe();
    case TOOLS_DISPENSER: return s_prepareDispenser();
    default: return 0;
  }
}

Tool getTool() {
  return s_tool;
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
}

int resetToolPreparations() {
  switch (s_tool) {
    case TOOLS_NONE:
      return 0;

    case TOOLS_PROBE:
      s_measuredReference = false;
      s_probeCalibrated = false;
      s_aligned = false;
      return 0;

    case TOOLS_DISPENSER:
      s_aligned = false;
      return 0;
  }
  return 0;
}

const char* toolTypeAsString(Tool tool) {
  switch(tool) {
    case TOOLS_NONE: return "none";
    case TOOLS_PROBE: return "probe";
    case TOOLS_DISPENSER: return "dispenser";
    default: return "unknown";
  }
}

int outputToolStatus() {
  SERIAL_ECHO("Tool");
  SERIAL_ECHO(" type:"); SERIAL_ECHO(toolTypeAsString(s_tool));
  SERIAL_ECHO("\n");

  SERIAL_ECHO("Homing");
  SERIAL_ECHO(" x:"); SERIAL_ECHO(getHomedState(X_AXIS));
  SERIAL_ECHO(" y:"); SERIAL_ECHO(getHomedState(Y_AXIS));
  SERIAL_ECHO(" z:"); SERIAL_ECHO(getHomedState(Z_AXIS));
  SERIAL_ECHO("\n");

  SERIAL_ECHO("Reference ");
  SERIAL_ECHO(" s_measuredReference:"); SERIAL_ECHO(s_measuredReference);
  SERIAL_ECHO(" s_probeCalibrated:"); SERIAL_ECHO(s_probeCalibrated);
  SERIAL_ECHO(" aligned:"); SERIAL_ECHO(s_aligned);
  SERIAL_ECHO(" reference.x:"); SERIAL_ECHO(s_reference.x);
  SERIAL_ECHO(" reference.y:"); SERIAL_ECHO(s_reference.y);
  SERIAL_PROTOCOL("\n");

  return 0;
}
