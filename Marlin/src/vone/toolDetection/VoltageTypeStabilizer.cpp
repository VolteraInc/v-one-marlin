#include "VoltageTypeStabilizer.h"

#include "../../../serial.h"
#include "classifyVoltage.h"

// static void outputVoltages(float voltages[], unsigned int num) {
//   SERIAL_ECHO_START;
//   SERIAL_ECHOPGM("  voltages = [");
//   serialArray(voltages, num);
//   SERIAL_ECHOPGM("]");
//   SERIAL_EOL;
// }

toolDetection::VoltageTypeStabilizer::VoltageTypeStabilizer() {
}

void toolDetection::VoltageTypeStabilizer::setStable(bool stable) {
  m_stable = stable;
}

void toolDetection::VoltageTypeStabilizer::add(float voltage) {
  auto type = classifyVoltage(voltage);

  // a (ring) buffer storing voltate and type will have a count (and current type)
  // ...so no need to have members for those
  // ...might as well wait for that

  if (m_type != type) {
    setStable(false);
    voltages.clear();
    m_type = type;
  }

  if (m_stable) {
    return;
  }

  // voltages.push({voltage, type});

  if (m_type == VoltageType::NoToolMounted || voltages.size() >= 2) {
    setStable(true);
    // TODO: output voltages
  }
}
