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

      inline void setProbeIsRetracting(bool probeIsRetracting) {
        if (m_probeIsRetracting == probeIsRetracting) {
          return;
        }

        m_probeIsRetracting = probeIsRetracting;

        // Probe no longer retracting, delay voltage readings a little longer
        if (!m_probeIsRetracting) {
          m_ignoreVoltagesUntil = millis() + 100;
        }
      }

    private:
      ToolBox& m_toolBox;
      const PTopPin& m_pin;
      volatile bool m_enabled = true;
      volatile bool m_probeIsRetracting = false;
      volatile unsigned long m_ignoreVoltagesUntil = 0;

      unsigned long m_nextCheckAt = 0;
      VoltageTypeStabilizer m_stabilizer;

      inline Tool* mapToTool(VoltageType type);
  };

  Tool* ToolDetector::mapToTool(VoltageType type) {
    switch (type) {
      // Probe
      case VoltageType::ProbeMounted:
      case VoltageType::ProbeTriggered:
        return &m_toolBox.probe;

      // Dispenser
      case VoltageType::SmartDispenserMounted:
        return &m_toolBox.smartDispenser;

      // Drill
      case VoltageType::DrillMounted:
        return &m_toolBox.drill;

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

    // The probe triggers intermittently while retracting, resulting in
    // unstable voltage readings. For simplicity, we ignore these readings.
    if (m_probeIsRetracting || now < m_ignoreVoltagesUntil) {
      return;
    }

    // Determine attached tool
    const auto sample = m_pin.value();
    m_stabilizer.add(sample.startTime, sample.voltage);
    const auto detectedTool = mapToTool(m_stabilizer.value());

    // Update tool
    if (detectedTool && m_enabled) {
      m_toolBox.setTool(detectedTool);
    }
  }
}
