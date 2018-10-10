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
  m_stepper.stop(F("tool detached"));
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


bool tools::Tool::baseCheckIfConnected(
  const VoltageLog& voltageLog,
  unsigned long time,
  float voltage,
  VoltageType& o_type
) {
  // No tool mounted, expect ~4.99
  if (voltage > 4.89) {
    o_type = VoltageType::NoToolMounted;
    return false;
  }

  // Voltage is stable, but was not recognized by subclass
  if (voltageLog.durationNear(voltage, 0.15) > 50) {
    o_type = VoltageType::NoToolMounted;
    return false;
  }

  // Unknown
  o_type = VoltageType::Unknown;
  return true;
}

VoltageType tools::Tool::classifyVoltage(
  const VoltageLog& voltageLog,
  unsigned long time,
  float voltage,
) {
  // No tool mounted, expect ~4.99
  if (voltage > 4.89) {
    return VoltageType::NoToolMounted;
  }

  // Voltage is stable, but was not recognized by subclass
  if (voltageLog.durationNear(voltage, 0.15) > 50) {
    return VoltageType::NoToolMounted;
  }

  // Unknown
  return VoltageType::Unknown;
}
