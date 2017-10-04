#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "../../vone/pins/PTopPin/PTopPin.h"
#include "internal.h"

// TODO: refactor Router into a class, and give it
//       a PTopPin& to eliminate dependency on VOne
#include "../../vone/VOne.h"

static int s_rotationSpeed = 0;

static uint8_t CRC8(uint8_t data) {
  uint8_t crc = 0x00;
  uint8_t extract = data;
  for (uint8_t tempI = 8; tempI; tempI--) {
    uint8_t sum = (crc ^ extract) & 0x01;
    crc >>= 1;
    if (sum) {
        crc ^= 0x8C;
    }
    extract>>=1;
  }
  return crc;
}

static int s_sendAndRampTo(int percent) {
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Set router rotation speed to "); SERIAL_ECHOLN(percent);

  // Format message
  char message[11];
  const int crc = CRC8(percent);
  sprintf(message, "R%u %u", percent, crc);

  // Send
  if (vone->pins.ptop.send(message)) {
    return -1;
  }

  // Give router time to ramp up (or down) to speed
  delay(Router::RampUpDuration);

  return 0;
}

int Router::prepare(Tool tool) {
  const char* context = "prepare router";
  return (
    raise() ||
    confirmMountedAndNotTriggered(context, tool, TOOLS_ROUTER) ||
    stopRotation(tool) ||

    ensureHomedInXY() ||
    homeZ(tool) || // home Z so we can enter the xy pos with decent precision
    centerTool(tool) ||
    homeZ(tool) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)

    raise()
  );
}

int Router::unprepare(Tool tool) {
  setHomedState(Z_AXIS, 0);
  return (
    stopRotationIfMounted(tool)
  );
}

float Router::getRotationSpeed(Tool tool) {
  if (tool != TOOLS_ROUTER) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Warning: rotation speed requested for "); SERIAL_ERROR(toolTypeAsString(tool));
    SERIAL_ECHOPGM(" returning "); SERIAL_ECHOLN(s_rotationSpeed);
  }
  return s_rotationSpeed;
}

// i.e. don't return an error if the tool is not mounted
int Router::stopRotationIfMounted(Tool tool) {
  // Finish pending moves before setting router speed.
  // Note: Whether we are starting, stopping or just changing speeds
  // having the change sync'd with movements makes it predictable.
  st_synchronize();

  bool isMounted = determineToolState(tool) == TOOL_STATE_ROUTER_MOUNTED;

  if (!isMounted){
    SERIAL_ECHO_START;
    SERIAL_ERRORLNPGM("Router could not be explicitly stopped because it is not mounted");
    goto DONE;
  }

  if (s_sendAndRampTo(0)) {
    bool stillMounted = determineToolState(tool) == TOOL_STATE_ROUTER_MOUNTED;
    if (stillMounted) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unable to stop router, confirm router is attached and powered");
      return -1;
    }

    SERIAL_ECHO_START;
    SERIAL_ERRORLNPGM("Router could not be explicitly stopped because it is not mounted");
    goto DONE;
  }

  // success
  SERIAL_ECHO_START;
  SERIAL_ERRORLNPGM("Router stopped");

DONE:
  s_rotationSpeed = 0;
  return 0;
}

int Router::stopRotation(Tool tool) {
  return setRotationSpeed(tool, 0);
}

int Router::setRotationSpeed(Tool tool, unsigned speed) {
  // Finish pending moves before setting router speed.
  // Note: Whether we are starting, stopping or just changing speeds
  // having the change sync'd with movements makes it predictable.
  st_synchronize();

  if (tool != TOOLS_ROUTER) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set router rotation speed, tool is "); SERIAL_ERRORLN(toolTypeAsString(tool));
    return -1;
  }

  if (speed > 100) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set rotation speed to "); SERIAL_ERROR(speed);
    SERIAL_ERRORLNPGM(" percent, value is outside expected range");
    return -1;
  }

  // Send the speed to the router
  if (s_sendAndRampTo(speed)) {
    SERIAL_ERROR_START;
    SERIAL_PAIR("Unable to set the router rotation speed to ", speed);
    SERIAL_ERRORLNPGM(", confirm router is attached and powered");

    // Attempt to stop the router (just in case)
    stopRotationIfMounted(tool);
    return -1;
  }

  // success
  s_rotationSpeed = speed;
  return 0;
}
