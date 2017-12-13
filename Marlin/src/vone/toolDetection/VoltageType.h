#pragma once

namespace toolDetection {

enum class VoltageType {
  Unknown,
  NoToolMounted,

  ProbeMounted,
  ProbeTriggered,

  RouterMounted,
  RouterResetting,
};

inline const char* toString(VoltageType type) {
  switch (type) {
    case VoltageType::ProbeTriggered: return "Triggered";
    case VoltageType::ProbeMounted: return "Probe Mounted";
    case VoltageType::RouterMounted: return "Router Mounted";
    case VoltageType::NoToolMounted: return "Not Mounted";
    case VoltageType::Unknown: return "Unknown";
  }
}

}
