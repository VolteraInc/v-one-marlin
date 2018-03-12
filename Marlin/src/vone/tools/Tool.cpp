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

int tools::Tool::prepareToMove(tools::PrepareToMove::Options options) {
  using Options = tools::PrepareToMove::Options;
  if (
    m_prepare_Completed ||
    (options == Options::startOnly && m_prepare_Started) ||
    (options == Options::eOnly && m_prepare_Started) ||
    (options == Options::skipCalibrateXYZ && m_prepare_HomedXY)
  ) {
    return 0;
  }

  m_prepare_Started = false;
  m_prepare_HomedXY = false;
  m_prepare_Completed = false;

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Preparing tool");

  // Start
  m_stepper.resume();
  if (prepareToMoveImpl_Start()) {
    return -1;
  }
  m_prepare_Started = true;

  // Return if that's all we need
  if (options == Options::startOnly) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Preparing tool -- completed start-only prepare");
    return 0;
  }
  if (options == Options::eOnly) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Preparing tool -- completed e-only prepare");
    return 0;
  }

  // Home XY
  if (prepareToMoveImpl_HomeXY()) {
    return -1;
  }
  m_prepare_HomedXY = true;

  // Calibrate XYZ
  if (options == Options::skipCalibrateXYZ) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Preparing tool -- skipping xyz-calibration");
    return 0;
  }
  if (prepareToMoveImpl_CalibrateXYZ()) {
    return -1;
  }

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Preparing tool -- completed full prepare");
  m_prepare_Completed = true;
  return 0;
}

int tools::Tool::resetPreparations() {
  if (prepared()) {
    m_prepare_Started = false;
    m_prepare_HomedXY = false;
    m_prepare_Completed = false;

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Reset tool preparations");
    return resetPreparationsImpl();
  }
  return 0;
}
