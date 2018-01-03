#include "Router.h"

#include "../../../serial.h"
#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../../api/api.h"
#include "../../vone/pins/PTopPin/PTopPin.h"

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

static int s_sendAndRampTo(PTopPin& pin, int percent) {
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Set router rotation speed to "); SERIAL_ECHOLN(percent);

  // Format message
  char message[11];
  const int crc = CRC8(percent);
  sprintf(message, "R%u %u", percent, crc);

  // Send
  if (pin.send(message)) {
    return -1;
  }

  // Give router time to ramp up (or down) to speed
  // NOTE: It takes 3s to ramp from 0 to max speed and
  //       we want some buffer too (hence the 500ul)
  const auto RampUpDuration = 3000ul;
  delay(RampUpDuration + 500ul);

  return 0;
}

tools::Router::Router(Stepper& stepper, PTopPin& pin)
  : Tool(stepper)
  , m_pin(pin)
{
}

int tools::Router::prepareToMoveImpl() {
  const char* context = "prepare router";
  return (
    raise() ||
    confirmAttached(context, *this) ||
    stopRotation() ||

    ensureHomedInXY(*this) ||
    homeZ(*this) || // home Z so we can enter the xy pos with decent precision
    centerTool(*this) ||
    homeZ(*this) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)

    raise()
  );
}

int tools::Router::resetPreparationsImpl() {
  setHomedState(Z_AXIS, 0);
  return (
    stopRotationIfMounted()
  );
}

// i.e. don't return an error if the tool is not mounted
int tools::Router::stopRotationIfMounted() {
  // Finish pending moves before setting router speed.
  // Note: Whether we are starting, stopping or just changing speeds
  // having the change sync'd with movements makes it predictable.
  st_synchronize();

  // TODO: better handling of resetting router
  // perhaps we should test pin directly?
  bool isMounted = determineToolState() == TOOL_STATE_ROUTER_MOUNTED;

  if (!isMounted) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Router could not be explicitly stopped because it is not mounted");
    goto DONE;
  }

  if (s_sendAndRampTo(m_pin, 0)) {
    bool stillMounted = determineToolState() == TOOL_STATE_ROUTER_MOUNTED;
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
  m_rotationSpeed = 0;
  return 0;
}

int tools::Router::enqueueMove(float x, float y, float z, float e, float f) {
  return asyncRawMove(x, y, z, e, f);
}

int tools::Router::stopRotation() {
  return setRotationSpeed(0);
}

int tools::Router::setRotationSpeed(unsigned speed) {
  // Finish pending moves before setting router speed.
  // Note: Whether we are starting, stopping or just changing speeds
  // having the change sync'd with movements makes it predictable.
  st_synchronize();

  if (confirmAttached("set router rotation speed", *this)) {
    return -1;
  }

  if (speed > 100) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set rotation speed to "); SERIAL_ERROR(speed);
    SERIAL_ERRORLNPGM(" percent, value is outside expected range");
    return -1;
  }

  // Send the speed to the router
  if (s_sendAndRampTo(m_pin, speed)) {
    SERIAL_ERROR_START;
    SERIAL_PAIR("Unable to set the router rotation speed to ", speed);
    SERIAL_ERRORLNPGM(", confirm router is attached and powered");

    // Attempt to stop the router (just in case)
    stopRotationIfMounted();
    return -1;
  }

  // success
  m_rotationSpeed = speed;
  return 0;
}
