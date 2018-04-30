
#include "../../../Marlin.h"
#include "../../../stepper.h"

#include "../api.h"
#include "../../vone/tools/Tool.h"

int asyncMove(tools::Tool& tool, float x, float y, float z, float e, float f) {
  return tool.enqueueMove(x, y, z, e, f);
}

int asyncRelativeMove(tools::Tool& tool, float x, float y, float z, float e, float speed_in_mm_per_min) {
  return asyncMove(
    tool,
    current_position[ X_AXIS ] + x,
    current_position[ Y_AXIS ] + y,
    current_position[ Z_AXIS ] + z,
    current_position[ E_AXIS ] + e,
    speed_in_mm_per_min
  );
}

int relativeMove(tools::Tool& tool, float x, float y, float z, float e, float speed_in_mm_per_min) {
  if (asyncRelativeMove(tool, x, y, z, e, speed_in_mm_per_min)) {
    return -1;
  }
  st_synchronize();

  // Check for endstop hits in each axis that we moved
  if (
    (x && endstop_triggered(X_AXIS)) ||
    (y && endstop_triggered(Y_AXIS)) ||
    (z && endstop_triggered(Z_AXIS))
  ) {
    // relying on endstop reporting and recovery at a higher-level
    log << F("Endstop hit during relative movement") << endl;
    return -1;
  }
  return 0;
}


int moveXY(tools::Tool& tool, float x, float y, float f) {
  if (asyncMove(tool, x, y, current_position[Z_AXIS], current_position[E_AXIS], f)) {
    return -1;
  }
  st_synchronize();

  // Check for endstop hits in X or Y-axis
  if (endstop_triggered(X_AXIS) || endstop_triggered(Y_AXIS)) {
    // relying on endstop reporting and recovery at a higher-level
    log << F("Endstop hit during x,y movement") << endl;
    return -1;
  }
  return 0;
}

int moveZ(tools::Tool& tool, float z, float f) {
  if (asyncMove(tool, current_position[X_AXIS], current_position[Y_AXIS], z, current_position[E_AXIS], f)) {
    return -1;
  }
  st_synchronize();

  // Check for an endstop hit in Z-axis
  if (endstop_triggered(Z_AXIS)) {
    // relying on endstop reporting and recovery at a higher-level
    log << F("Endstop hit during z movement") << endl;
    return -1;
  }
  return 0;
}

int confirmAttached(const char* context, tools::Tool& tool) {
  if (tool.detached()) {
    logError
      << F("Unable to ") << context
      << F(", tool type '") << tool.name()
      << F("' is required")
      << endl;
    return -1;
  }
  return 0;
}

int ensureHomedInXY(tools::Tool& tool) {
  if (homedXY()) {
    return 0;
  }
  return homeXY(tool);
}

int centerTool(tools::Tool& tool) {
  float toolX, toolY;
  if (xyPositionerFindCenter(tool, defaultXyPositionerCycles, toolX, toolY)) {
    return -1;
  }

  auto const dx = abs(xypos_x_pos - toolX);
  auto const dy = abs(xypos_y_pos - toolY);
  if (dx > 1 || dy > 1) {
    logError
      << F("Calculated XY center is very different than calibrated. ")
      << F("Calibrated X: ") << xypos_x_pos
      << F(" Y: ") << xypos_y_pos
      << F(" Measured X: ") << toolX
      << F(" Y: ") << toolY
      << endl;
    return -1;
  }

  // Overwrite the current position with constant position.
  setPosition(xypos_x_pos, xypos_y_pos, current_position[Z_AXIS], current_position[E_AXIS]);
  return 0;
}

int retractToolConditionally(float distance, float additionalRetractDistance) {
  if (additionalRetractDistance == NoRetract) {
    return 0;
  }
  return retractFromSwitch(Z_AXIS, -1, distance + additionalRetractDistance);
}
