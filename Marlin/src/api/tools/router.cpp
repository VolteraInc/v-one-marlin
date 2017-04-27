#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"

static float s_rotationSpeed = 0.0f;

int prepareRouter(Tool tool) {
  const char* context = "prepare probe";
  return (
    setRotationSpeed(tool, 0.0f) ||
    raise() ||
    confirmMountedAndNotTriggered(context, tool, TOOLS_ROUTER) ||
    ensureHomedInXY() ||
    ensureHomedInZ(tool) ||
    centerTool(tool) ||
    raise()
  );
}

float getRotationSpeed(Tool tool) {
  if (tool != TOOLS_ROUTER) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Warning: rotation speed requested for "); SERIAL_ERROR(toolTypeAsString(tool));
    SERIAL_ECHOPGM(" returning "); SERIAL_ECHOLN(s_rotationSpeed);
  }
  return s_rotationSpeed;
}

int setRotationSpeed(Tool tool, float speed) {
  if (logging_enabled) {
    SERIAL_ECHOPGM("Setting rotation speed to "); SERIAL_ECHO(speed);
    SERIAL_ECHOPGM("units\n");
  }

  if (tool != TOOLS_ROUTER) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set router rotation speed, tool is "); SERIAL_ERRORLN(toolTypeAsString(tool));
    return -1;
  }

  if (speed == 0.0f) {
    s_rotationSpeed = speed;
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("TODO: Stop the router");

  } else if ( speed >= 80.0f || speed <= 120.0f) {
    s_rotationSpeed = speed;
    SERIAL_ECHO_START;
    SERIAL_ECHO("TODO: Set rotation speed to "); SERIAL_ECHOLN(s_rotationSpeed);

  } else {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set rotation speed to "); SERIAL_ERROR(speed);
    SERIAL_ERRORPGM("units, value is outside expected range\n");
    return -1;
  }

  return 0;
}
