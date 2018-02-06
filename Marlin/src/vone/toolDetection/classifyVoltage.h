#pragma once

#include "VoltageType.h"

namespace toolDetection {

inline VoltageType classifyVoltage(float voltage) {
  // Probe - Triggered, expect ~0.07
  if (voltage < 0.08) {
    return VoltageType::ProbeTriggered;

  // Router - Resetting, expect ~0.39
  } else if (voltage >= 0.29 && voltage <= 0.49) {
    return VoltageType::RouterResetting;

  // Router, expect ~1.31
  } else if (voltage >= 1.21 && voltage <= 1.41) {
    return VoltageType::RouterMounted;

  // Probe, expect ~3.47
  } else if (voltage >= 3.25 && voltage <= 3.75) {
    return VoltageType::ProbeMounted;

  // No tool mounted, expect ~4.99
  } else if (voltage > 4.89) {
    return VoltageType::NoToolMounted;

  // Unknown, voltage is likely climbing
  } else {
    return VoltageType::Unknown;
  }
}

}
