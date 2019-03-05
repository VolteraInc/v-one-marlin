#pragma once

#include "VoltageType.h"

namespace toolDetection {

inline VoltageType classifyVoltage(float voltage) {
  // Probe - Triggered, expect ~0.07
  if (voltage < 0.08) {
    return VoltageType::ProbeTriggered;

  // Drill - Communicating OR Resetting, expect 0.39 ~ 0.43
  } else if (voltage >= 0.29 && voltage <= 0.53) {
    return VoltageType::DrillMounted;

  // Drill - Mounted, expect 1.31 ~ 1.34
  } else if (voltage >= 1.21 && voltage <= 1.44) {
    return VoltageType::DrillMounted;

  // Probe, expect ~3.47
  } else if (voltage >= 3.25 && voltage <= 3.75) {
    return VoltageType::ProbeMounted;

  } else if (voltage >= 4.25 && voltage <= 4.75) {
    return VoltageType::SmartDispenserMounted;

  // No tool mounted, expect ~4.99
  } else if (voltage > 4.89) {
    return VoltageType::NoToolMounted;

  // Unknown, voltage is likely climbing
  } else {
    return VoltageType::Unknown;
  }
}

}
