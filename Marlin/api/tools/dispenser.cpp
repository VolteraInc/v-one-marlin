#include "../../Marlin.h"
#include "../../planner.h"
#include "../api.h"
#include "internal.h"

static float s_dispenseHeight = 0.0f;

int prepareDispenser(Tool tool) {
  if (raise()) {
    return -1;
  }

  if(probeMounted()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to prepare dispenser, the probe is mounted");
    return -1;
  }

  // Wiggle E to mesh gears.
  if (meshGears()) {
    return -1;
  }

  // Ensure homed in Z
  if (!homedZ()) {
    if (homeZ(tool)) {
      return -1;
    }
  }

  Point2D toolPosition;
  if (xyPositionerFindCenter(tool, defaultXyPositionerCycles, toolPosition.x, toolPosition.y)) {
    return -1;
  }

  auto const dx = abs(xypos_x_pos - toolPosition.x);
  auto const dy = abs(xypos_y_pos - toolPosition.y);

  if(dx > 1 || dy > 1){
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Calculated XY center is very different than calibrated. ");
    SERIAL_ERRORPGM("Calibrated X: "); SERIAL_ERROR(xypos_x_pos);
    SERIAL_ERRORPGM(" Y: "); SERIAL_ERROR(xypos_y_pos);
    SERIAL_ERRORPGM(" Measured X: "); SERIAL_ERROR(toolPosition.x);
    SERIAL_ERRORPGM(" Y: "); SERIAL_ERRORLN(toolPosition.y);
    return -1;
  }

  // Overwrite the current position with constant position.
  setPosition(xypos_x_pos, xypos_y_pos, current_position[Z_AXIS], current_position[E_AXIS]);

  if (raise()) {
    return -1;
  }
  return 0;
}

float getDispenseHeight(Tool tool) {
  // Confirm we have a dispenser
  if (tool != TOOLS_DISPENSER) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Warning: dispense height requested for "); SERIAL_ERROR(toolTypeAsString(getTool()));
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
    SERIAL_ERRORPGM("Unable to set dispensing height, current tool is "); SERIAL_ERRORLN(toolTypeAsString(getTool()));
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
