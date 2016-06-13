#include "../../Marlin.h"
#include "../../planner.h"
#include "../api.h"
#include "internal.h"

static float s_dispenseHeight = dispenseHeightUpperBound;

static int s_alignToReference(Tool tool, const Point2D& reference) {
  // Move the current tool to the reference position
  // i.e. the center of the xy-positioner.
  Point2D toolPosition;
  if (xyPositionerFindCenter(tool, defaultXyPositionerCycles, toolPosition.x, toolPosition.y)) {
    return -1;
  }

  // The tip of the tool is currently at the center of the xy-positioner
  // Override the current position
  current_position[X_AXIS] = reference.x;
  current_position[Y_AXIS] = reference.y;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  return 0;
}

int prepareDispenser(Tool tool, const Point2D& reference) {
  // Ensure homed in Z
  if (!homedZ()) {
    if (homeZ(tool)) {
      return -1;
    }
  }

  // Measure the center of the xy-positioner to compensate for the dispenser's offset
  if (s_alignToReference(tool, reference)) {
    return -1;
  }

  if (raise()) {
    return -1;
  }
  return 0;
}

float getDispenseHeight(Tool tool) {
  // Confirm we have a dispenser
  if (tool != TOOLS_DISPENSER) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to get dispensing height, current tool is "); SERIAL_ERRORLN(toolTypeAsString(getTool()));
    return 5;
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
    SERIAL_ERRORPGM("Unable to set dispensing height, current tool is "); SERIAL_ERRORLN(toolTypeAsString(getTool()));
    return -1;
  }

  if (height < 0.0f || height > dispenseHeightUpperBound ) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set dispensing height to "); SERIAL_ERROR(height);
    SERIAL_ERRORPGM("mm, value is outside expected range\n");
    return -1;
  }

  // Set the height
  s_dispenseHeight = height;

  return 0;
}
