#include "../../../Marlin.h"
#include "../../../stepper.h"

#include "../api.h"
#include "../internal.h"

#include "internal.h"

static Tool s_tool = TOOLS_NONE;

static bool s_toolPrepared = false;

void sendToolStatusUpdate() {
  SERIAL_PROTOCOLPGM("toolUpdate");
  SERIAL_PROTOCOLPGM(" type:"); SERIAL_PROTOCOL(toolTypeAsString(s_tool));
  SERIAL_PROTOCOLPGM("\n");
}

static int s_prepareTool(Tool tool) {
  switch (tool) {
    case TOOLS_PROBE: return prepareProbe(tool);
    case TOOLS_DISPENSER: return prepareDispenser(tool);
    case TOOLS_ROUTER: return prepareRouter(tool);
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

void setTool(Tool tool) {
  if (s_tool == tool) {
    return;
  }
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Swapping "); SERIAL_ECHO(toolTypeAsString(s_tool));
  SERIAL_ECHOPGM(" for "); SERIAL_ECHOLN(toolTypeAsString(tool));
  resetToolPreparations();
  s_tool = tool;
  sendToolStatusUpdate();
}

Tool getTool() {
  return s_tool;
}

int resetToolPreparations() {
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Reset tool preparations");
  s_toolPrepared = false;
  enable_p_top(false);
  setDispenseHeight(TOOLS_DISPENSER, 0.0f);
  if (getTool() == TOOLS_ROUTER) {
    setRotationSpeed(TOOLS_ROUTER, 0);
  }
  setHomedState(Z_AXIS, 0);
  return 0;
}

const char* toolTypeAsString(Tool tool) {
  switch(tool) {
    case TOOLS_NONE: return "None";
    case TOOLS_PROBE: return "Probe";
    case TOOLS_DISPENSER: return "Dispenser";
    case TOOLS_ROUTER: return "Router";
    default: return "unknown";
  }
}

int outputToolStatus() {
  const auto toolState = determineToolState(s_tool);
  SERIAL_ECHOPGM("Tool\n");
  SERIAL_ECHOPGM("  type: "); SERIAL_ECHOLN(toolTypeAsString(s_tool));

  SERIAL_ECHOPGM("Probe\n");
  SERIAL_ECHOPGM("  status: "); SERIAL_ECHOLN(
    toolStateAsString(s_tool == TOOLS_PROBE ? toolState : TOOL_STATE_NOT_MOUNTED)
  );

  SERIAL_ECHOPGM("Dispenser\n");
  SERIAL_ECHOPGM("  dispense height: "); SERIAL_ECHOLN(getDispenseHeight(TOOLS_DISPENSER));

  SERIAL_ECHOPGM("Router\n");
  SERIAL_ECHOPGM("  status: "); SERIAL_ECHOLN(
    toolStateAsString(s_tool == TOOLS_ROUTER ? toolState : TOOL_STATE_NOT_MOUNTED)
  );
  SERIAL_ECHOPGM("  Speed: "); SERIAL_ECHOLN(getRotationSpeed(TOOLS_ROUTER));

  SERIAL_ECHOPGM("Homing\n");
  SERIAL_ECHOPGM("  x: "); SERIAL_ECHOLN(getHomedState(X_AXIS));
  SERIAL_ECHOPGM("  y: "); SERIAL_ECHOLN(getHomedState(Y_AXIS));
  SERIAL_ECHOPGM("  z: "); SERIAL_ECHOLN(getHomedState(Z_AXIS));

  SERIAL_ECHOPGM("Status\n");
  SERIAL_ECHOPGM("  Prepared: "); SERIAL_ECHOLN(s_toolPrepared);

  return 0;
}

int asyncMove(Tool tool, float x, float y, float z, float e, float f, bool applyDispenseHeight) {
  if (applyDispenseHeight) {
    z += getDispenseHeight(tool);
  }

  return asyncRawMove(x, y, z, e, f);
}

int asyncRelativeMove(Tool tool, float x, float y, float z, float e, float speed_in_mm_per_min) {
  return asyncMove(
    tool,
    current_position[ X_AXIS ] + x,
    current_position[ Y_AXIS ] + y,
    current_position[ Z_AXIS ] + z,
    current_position[ E_AXIS ] + e,
    speed_in_mm_per_min
  );
}

int relativeMove(Tool tool, float x, float y, float z, float e, float speed_in_mm_per_min) {
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
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Endstop hit during relative movement");
    return -1;
  }
  return 0;
}


int moveXY(Tool tool, float x, float y, float f) {
  if (asyncMove(tool, x, y, current_position[Z_AXIS], current_position[E_AXIS], f)) {
    return -1;
  }
  st_synchronize();

  // Check for endstop hits in X or Y-axis
  if (endstop_triggered(X_AXIS) || endstop_triggered(Y_AXIS)) {
    // relying on endstop reporting and recovery at a higher-level
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Endstop hit during x,y movement");
    return -1;
  }
  return 0;
}

int moveZ(Tool tool, float z, float f, bool applyDispenseHeight) {
  if (asyncMove(tool, current_position[X_AXIS], current_position[Y_AXIS], z, current_position[E_AXIS], f, applyDispenseHeight)) {
    return -1;
  }
  st_synchronize();

  // Check for an endstop hit in Z-axis
  if (endstop_triggered(Z_AXIS)) {
    // relying on endstop reporting and recovery at a higher-level
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Endstop hit during z movement");
    return -1;
  }
  return 0;
}

int confirmMountedAndNotTriggered(const char* context, Tool tool, Tool requiredTool) {
  if (tool != requiredTool) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to "); SERIAL_ERROR(context); SERIAL_ERRORPGM(", ");
    if (requiredTool == TOOLS_NONE) {
      SERIAL_ERROR(toolTypeAsString(tool)); SERIAL_ERRORLNPGM(" is attached");
    } else {
      SERIAL_ERROR(toolTypeAsString(requiredTool)); SERIAL_ERRORLNPGM(" not attached");
    }
    return -1;
  }

  switch (determineToolState(tool)) {
    case TOOL_STATE_PROBE_MOUNTED:
      if (tool == TOOLS_PROBE) {
        return 0;
      }

      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to "); SERIAL_ERROR(context);
      SERIAL_ERRORLNPGM(", probe is mounted");
      return -1;

    case TOOL_STATE_ROUTER_MOUNTED:
      if (tool == TOOLS_ROUTER) {
        return 0;
      }

      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to "); SERIAL_ERROR(context);
      SERIAL_ERRORLNPGM(", router is mounted");
      return -1;

    case TOOL_STATE_NOT_MOUNTED:
      // Note: The dispense does not contact the pogo pins so it shows as 'Not Mounted'
      if (tool == TOOLS_DISPENSER || tool == TOOLS_NONE) {
        return 0;
      }
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to "); SERIAL_ERROR(context); SERIAL_ERRORLNPGM(", no tool mounted");
      return -1;

    case TOOL_STATE_TRIGGERED:
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to "); SERIAL_ERROR(context); SERIAL_ERRORLNPGM(", tool reported contact before movement started");
      return -1;

    default:
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to "); SERIAL_ERROR(context); SERIAL_ERRORLNPGM(", could not determine if tool mounted");
      return -1;
  }
}

int ensureHomedInXY() {
  if (homedXY()) {
    return 0;
  }
  return homeXY();
}

int centerTool(Tool tool) {
  float toolX, toolY;
  if (xyPositionerFindCenter(tool, defaultXyPositionerCycles, toolX, toolY)) {
    return -1;
  }

  auto const dx = abs(xypos_x_pos - toolX);
  auto const dy = abs(xypos_y_pos - toolY);
  if (dx > 1 || dy > 1) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Calculated XY center is very different than calibrated. ");
    SERIAL_ERRORPGM("Calibrated X: "); SERIAL_ERROR(xypos_x_pos);
    SERIAL_ERRORPGM(" Y: "); SERIAL_ERROR(xypos_y_pos);
    SERIAL_ERRORPGM(" Measured X: "); SERIAL_ERROR(toolX);
    SERIAL_ERRORPGM(" Y: "); SERIAL_ERRORLN(toolY);
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
