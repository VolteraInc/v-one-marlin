#include "Tool.h"

#include "../../../serial.h"
#include "../stepper/Stepper.h"

tools::Tool::Tool(Stepper& stepper)
  : m_stepper(stepper)
{
}

void tools::Tool::attach() {
  m_attached = true;
}

void tools::Tool::detach() {
  m_attached = false;
  m_stepper.stop();
  resetPreparations();
}

int tools::Tool::prepareToMove(tools::PrepareToMove::Option option) {
  using Option = tools::PrepareToMove::Option;
  if (
    m_prepare_Completed ||
    (option == Option::startOnly && m_prepare_Started) ||
    (option == Option::eOnly && m_prepare_Started) ||
    (option == Option::skipCalibrateXYZ && m_prepare_HomedXY)
  ) {
    return 0;
  }

  m_prepare_Started = false;
  m_prepare_HomedXY = false;
  m_prepare_Completed = false;

  log << F("Preparing tool") << endl;

  // Start
  m_stepper.resume();
  if (prepareToMoveImpl_Start()) {
    return -1;
  }
  m_prepare_Started = true;

  // Return if that's all we need
  if (option == Option::startOnly) {
    log << F("Preparing tool -- completed start-only prepare") << endl;
    return 0;
  }
  if (option == Option::eOnly) {
    log << F("Preparing tool -- completed e-only prepare") << endl;
    return 0;
  }

  // Home XY
  if (prepareToMoveImpl_HomeXY()) {
    return -1;
  }
  m_prepare_HomedXY = true;

  // Calibrate XYZ
  if (option == Option::skipCalibrateXYZ) {
    log << F("Preparing tool -- skipping xyz-calibration") << endl;
    return 0;
  }
  if (prepareToMoveImpl_CalibrateXYZ()) {
    return -1;
  }

  log << F("Preparing tool -- completed full prepare") << endl;
  m_prepare_Completed = true;
  return 0;
}

int tools::Tool::resetPreparations() {
  log << F("Reset tool preparations") << endl;
  m_prepare_Started = false;
  m_prepare_HomedXY = false;
  m_prepare_Completed = false;
  return resetPreparationsImpl();
}
