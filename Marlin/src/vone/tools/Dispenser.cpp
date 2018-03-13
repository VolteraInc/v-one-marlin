#include "Dispenser.h"

#include "../../../Marlin.h" // Z_AXIS
#include "../../api/api.h"

tools::Dispenser::Dispenser(Stepper& stepper)
  : tools::Tool(stepper)
{
}

int tools::Dispenser::meshGears() {
  // Move E backward, then move forward.
  // NOTE: we have seen 0.02 fail to mesh the gears so we are trying 0.06
  return (
    relativeRawMoveE(-0.06) ||
    relativeRawMoveE(0.06)
  );
}

int tools::Dispenser::prepareToMoveImpl_Start() {
  return (
    raise() ||
    confirmAttached("prepare dispenser", *this) ||
    meshGears()
  );
}

int tools::Dispenser::prepareToMoveImpl_HomeXY() {
  return ensureHomedInXY(*this);
}

int tools::Dispenser::prepareToMoveImpl_CalibrateXYZ() {
  return (
    homeZ(*this) || // home Z so we can enter the xy pos with decent precision
    centerTool(*this) ||
    homeZ(*this) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)
    raise()
  );
}

int tools::Dispenser::resetPreparationsImpl() {
  setHomedState(Z_AXIS, 0);
  return (
    setDispenseHeight(0.0f)
  );
}

float tools::Dispenser::dispenseHeight() const {
  return m_dispenseHeight;
}

int tools::Dispenser::setDispenseHeight(float height) {
  if (height < 0.0f || height > 2.0f ) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set dispensing height to "); SERIAL_ERROR(height);
    SERIAL_ERRORPGM("mm, value is outside expected range\n");
    return -1;
  }

  // Set the height
  m_dispenseHeight = height;

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_PAIR("Dispense height set to ", m_dispenseHeight);
    SERIAL_ECHOPGM("mm");
    SERIAL_EOL;
  }

  return 0;
}


int tools::Dispenser::enqueueMove(float x, float y, float z, float e, float f) {
  // TODO: if moving from 0,0, check z-height
  return asyncRawMove(x, y, z, e, f);
}

int tools::Dispenser::enqueueDispense(float x, float y, float z, float e, float f) {
  return enqueueMove(x, y, z + dispenseHeight(), e, f);
}
