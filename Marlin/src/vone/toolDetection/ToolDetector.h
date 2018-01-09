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
      unsigned int m_nextCheckAt = 0;
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
    return nullptr;
  }

  void ToolDetector::frequentInterruptibleWork() {
    // Run periodically
    // Note: Voltage climbs from 0 to 5 over ~60ms,
    //       Collecting every ~20ms should be sufficient
    //       for tool classification and detach detection.
    //       In the worst case it should take 3-4 samples.
    const auto now = millis();
    if (now < m_nextCheckAt) {
      return;
    }
    m_nextCheckAt = now + 20;

    // Determine attached tool
    auto sample = m_pin.value();
    m_stabilizer.add(sample.startTime, sample.voltage);
    const auto detectedTool = mapToTool(m_stabilizer.value());

    // Update tool
    if (detectedTool && m_enabled) {
      m_toolBox.setTool(detectedTool);
    }
  }
}
