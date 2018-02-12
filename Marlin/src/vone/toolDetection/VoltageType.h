#pragma once

namespace toolDetection {

enum class VoltageType {
  Unknown,
  NoToolMounted,

  ProbeMounted,
  ProbeTriggered,

  DrillMounted,
  DrillResetting,
};

inline const char* toString(VoltageType type) {
  switch (type) {
    case VoltageType::Unknown: return "Unknown";
    case VoltageType::NoToolMounted: return "Not Mounted";

    case VoltageType::ProbeMounted: return "Probe Mounted";
    case VoltageType::ProbeTriggered: return "Triggered";

    case VoltageType::DrillMounted: return "Drill Mounted";
    case VoltageType::DrillResetting: return "Drill Resetting";
  }
  return "Unknown";
}

}
