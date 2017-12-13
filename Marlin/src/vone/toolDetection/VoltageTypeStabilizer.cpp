#pragma once

#include "VoltateTypeStabilizer.h"

#include "../../../Marlin.h"
#include "classifyVoltage.h"

static void outputVoltages(float voltages, unsigned int num) {
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("  voltages = [");
  serialArray(voltages, num);
  SERIAL_ECHOPGM("]");
  SERIAL_EOL;
}

VoltateTypeStabilizer::VoltateTypeStabilizer() {
  reset();
}

VoltateTypeStabilizer::reset() {
  m_type = VoltageType::Unknown;
  m_numVoltages = 0;

}

VoltateTypeStabilizer::addVoltage(float voltage) {
  if (m_numVoltages < maxVoltages) {
    voltages[numVoltages] = sample.voltage;
    ++numVoltages;
  }
}

void VoltateTypeStabilizer::setStable(bool stable) {
  if (m_stable == stable) {
    return;
  }
  m_stable = stable;

  if (m_stable) {
  } else {
    start = millis();
  }
}

void VoltateTypeStabilizer::add(PTopPin::Sample& sample) {

  const delta = m_previousTime - sample.startTime;

  // Given sample is same as previous, can't make progress, return
  if (delta == 0) {
    return;
  }

  // Given sample occurred significantly after previous, reset count
  if (delta >= 500) {
    numVoltages = 0;
    count = 0;
  }

  m_previousTime = sample.startTime;

  addVoltage(sample.voltage);
  const auto type = classifyVoltage(sample.voltage);

  // type differs from previous, reset counter
  if (m_previousType != type) {
    reset();
  }
  previousType = type;

  // Set to stable when we collect enough matching
  // voltage type classications
  if (!m_stable && ++m_count >= 4) {
    setType(type);
  }
}

VoltageType VoltageTypeStabilizer::value() {
  return m_type;
}