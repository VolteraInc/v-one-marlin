#include "Drill.h"

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
  log << F("Set drill rotation speed to ") << percent << endl;

  // Format message
  char message[11];
  const int crc = CRC8(percent);
  sprintf(message, "R%u %u", percent, crc);

  // Send
  if (pin.send(message)) {
    return -1;
  }

  // Give drill time to ramp up (or down) to speed
  // NOTE: It takes 3s to ramp from 0 to max speed and
  //       we want some buffer too (hence the 500ul)
  const auto RampUpDuration = 3000ul;
  delay(RampUpDuration + 500ul);

  return 0;
}

tools::Drill::Drill(Stepper& stepper, PTopPin& pin)
  : Tool(stepper)
  , m_pin(pin)
{
}

int tools::Drill::prepareToMoveImpl_Start() {
  return (
    raiseToEndstop() ||
    confirmAttached("prepare drill", *this) ||
    stopRotation()
  );
}

int tools::Drill::prepareToMoveImpl_HomeXY() {
  return ensureHomedInXY(*this);
}

int tools::Drill::prepareToMoveImpl_CalibrateXYZ() {
  return (
    homeZOnly(*this) || // home Z so we can enter the xy pos with decent precision
    centerTool(*this) ||
    homeZandEstablishSoftMax(*this) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)
    raiseToSoftMax(*this)
  );
}

int tools::Drill::resetPreparationsImpl() {
  setHomedState(Z_AXIS, 0);
  return (
    stopRotationIfMounted()
  );
}

// i.e. don't return an error if the tool is not mounted
int tools::Drill::stopRotationIfMounted() {
  // Finish pending moves before setting drill speed.
  // Note: Whether we are starting, stopping or just changing speeds
  // having the change sync'd with movements makes it predictable.
  st_synchronize();

  if (detached()) {
    goto DETACHED;
  }

  if (s_sendAndRampTo(m_pin, 0)) {
    if (detached()) {
      goto DETACHED;
    }

    logError << F("Unable to stop drill, confirm drill is attached and powered") << endl;
    return -1;
  }

  // success
  log << F("Drill stopped") << endl;
  goto DONE;

DETACHED:
  log << F("Drill could not be explicitly stopped because it is not mounted") << endl;

DONE:
  m_rotationSpeed = 0;
  return 0;
}

int tools::Drill::enqueueMove(float x, float y, float z, float e, float f) {
  return asyncRawMove(x, y, z, e, f);
}

int tools::Drill::stopRotation() {
  return setRotationSpeed(0);
}

int tools::Drill::setRotationSpeed(unsigned speed) {
  // Finish pending moves before setting drill speed.
  // Note: Whether we are starting, stopping or just changing speeds
  // having the change sync'd with movements makes it predictable.
  st_synchronize();

  if (confirmAttached("set drill rotation speed", *this)) {
    return -1;
  }

  if (speed > 100) {
    logError
      << F("Unable to set rotation speed to ") << speed
      << F(" percent, value is outside expected range")
      << endl;
    return -1;
  }

  // Send the speed to the drill
  if (s_sendAndRampTo(m_pin, speed)) {
    logError
      << F("Unable to set the drill rotation speed to ") << speed
      << F(", confirm drill is attached and powered")
      << endl;

    // Attempt to stop the drill (just in case)
    stopRotationIfMounted();
    return -1;
  }

  // success
  m_rotationSpeed = speed;
  return 0;
}
