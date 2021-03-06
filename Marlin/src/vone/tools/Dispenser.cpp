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
    raiseToEndstop() ||
    confirmAttached("prepare dispenser", *this) ||
    meshGears()
  );
}

int tools::Dispenser::prepareToMoveImpl_HomeXY() {
  return ensureHomedInXY(*this);
}

int tools::Dispenser::prepareToMoveImpl_CalibrateXYZ() {
  return (
    homeZOnly(*this) || // home Z so we can enter the xy pos with decent precision
    centerTool(*this) ||
    homeZandEstablishSoftMax(*this) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)
    raiseToSoftMax(*this)
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
    logError
      << F("Unable to set dispensing height to ") << height
      << F("mm, value is outside expected range")
      << endl;
    return -1;
  }

  // Set the height
  m_dispenseHeight = height;

  if (logging_enabled) {
    log << F("Dispense height set to ") << m_dispenseHeight << F("mm") << endl;
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
