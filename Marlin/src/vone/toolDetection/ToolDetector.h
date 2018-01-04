#pragma once

// #include "../../../macros.h"
#include "../tools/ToolBox.h"
#include "../pins/PTopPin/PTopPin.h"

#include "VoltageType.h"
#include "VoltageTypeStabilizer.h"

namespace toolDetection {

  using Tool = tools::Tool;
  using ToolBox = tools::ToolBox;

  class ToolDetector {

    public:
      ToolDetector(ToolBox& tools, const PTopPin& pin)
        : m_toolBox(tools)
        , m_pin(pin)
      {
      }

      bool enabled() const { return m_enabled; }
      void enable(bool enable) { m_enabled = enable; }

      inline void frequentInterruptibleWork();

    private:
      ToolBox& m_toolBox;
      const PTopPin& m_pin;
      volatile bool m_enabled = true;
      VoltageTypeStabilizer m_stabilizer;

      inline Tool* mapToTool(VoltageType type);
  };

  Tool* ToolDetector::mapToTool(VoltageType type) {
    switch (type) {
      // Probe
      case VoltageType::ProbeMounted:
      case VoltageType::ProbeTriggered:
        return &m_toolBox.probe;

      // Router
      case VoltageType::RouterMounted:
        return &m_toolBox.router;
      case VoltageType::RouterResetting:
        return &m_toolBox.nullTool;  // treat resets as detaches

      // Dispenser and NoTool
      case VoltageType::NoToolMounted:
        if (m_toolBox.dispenser.attached()) {
          return &m_toolBox.dispenser;
        } else {
          return &m_toolBox.nullTool;
        }

      // Unknown
      case VoltageType::Unknown:
        return nullptr;
    }
  }

  void ToolDetector::frequentInterruptibleWork() {
    // Keep current tool up-to-date
    if (m_enabled) {
      m_stabilizer.add(m_pin.value());
      const auto detectedTool = mapToTool(m_stabilizer.value());
      if (detectedTool) {
        m_toolBox.setTool(detectedTool);
      }
    }
  }
}
