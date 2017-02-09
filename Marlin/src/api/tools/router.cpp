#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"
#include <SoftwareSerial.h>

// ----------------------------------------------
// Router - serial comms

static const int Tx = A2;
static const int Rx = A3;
static SoftwareSerial s_router(Rx, Tx);

static void ensureInitialized() {
  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    SET_INPUT(ROUTER_COMMS_PIN);

    // baudrate of 300 is based on the rise and fall times of the capacitor on ptop
    s_router.begin(300);
  }
}

static void readMode() { SET_INPUT(ROUTER_COMMS_PIN); }
static void writeMode() { SET_OUTPUT(ROUTER_COMMS_PIN); }

static void s_setRouterRotationSpeed(int rpm) {
  const int pulsewidth = rpm;
  SERIAL_ECHO_START;
  SERIAL_ECHO("Set rotation speed to "); SERIAL_ECHOLN(rpm);

  // change to write mode
  ensureInitialized();
  writeMode();
  s_router.write(pulsewidth);
  readMode();
}


// ----------------------------------------------
// Router - tool
static float s_rotationSpeed = 0.0f;

int prepareRouter(Tool tool) {
  const char* context = "prepare probe";
  return (
    setRotationSpeed(tool, 0.0f) ||
    raise() ||
    confirmRequiredToolAttached(context, tool, TOOLS_ROUTER) ||
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
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("Stop the router");
    s_rotationSpeed = 0.0f;
    s_setRouterRotationSpeed(s_rotationSpeed);

  } else if ( speed >= 80.0f || speed <= 120.0f) {
    s_setRouterRotationSpeed(s_rotationSpeed);

  } else {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set rotation speed to "); SERIAL_ERROR(speed);
    SERIAL_ERRORPGM("units, value is outside expected range\n");
    return -1;
  }

  return 0;
}
