#pragma once

#include "../../../macros.h"
#include "../tools/Tool.h"
#include "../tools/ToolBox.h"
#include "../stepper/Stepper.h"
#include "../pins/PTopPin/PTopPin.h"

#include "VoltageType.h"
#include "VoltageTypeStabilizer.h"

namespace toolDetection {

  using Tool = tools::Tool;
  using ToolBox = tools::ToolBox;

  class ToolDetector {

    public:
      ToolDetector(ToolBox& tools, Stepper& stepper, const PTopPin& pin);

      bool enabled() const { return m_enabled; }
      void enable(bool enable) { m_enabled = enable; }

      inline void setTool(Tool* newTool);
      Tool* currentTool() const { return m_currentTool; }
      void updateTool();

      inline void frequentInterruptibleWork();

      void outputToolStatus() const;

    private:
      ToolBox& m_tools;
      Stepper& m_stepper;
      const PTopPin& m_pin;
      volatile bool m_enabled = true;
      volatile Tool* m_detectedTool = nullptr;
      Tool* m_currentTool = nullptr;
      VoltageTypeStabilizer m_stabilizer;

      inline Tool* mapToTool(VoltageType type);
  };

  Tool* ToolDetector::mapToTool(VoltageType type) {
    switch (type) {
      // Probe
      case VoltageType::ProbeMounted:
      case VoltageType::ProbeTriggered:
        return &m_tools.probe;

      // Router
      // Note: we treat resets as detaches
      case VoltageType::RouterMounted:
        return &m_tools.router;
      case VoltageType::RouterResetting:
        return &m_tools.nullTool;

      // Dispenser and NoTool
      case VoltageType::NoToolMounted:
        return m_tools.dispenser.attached() ? &m_tools.dispenser : &m_tools.nullTool;

      // Unknown
      case VoltageType::Unknown:
        return m_currentTool;
    }
  }

  void ToolDetector::frequentInterruptibleWork() {
    // Keep current tool up-to-date
    if (m_enabled) {
      m_stabilizer.add(m_pin.value());
      m_detectedTool = mapToTool(m_stabilizer.value());
      if (m_detectedTool != m_currentTool) {
        m_stepper->stop();
      }
    }
  }
}
