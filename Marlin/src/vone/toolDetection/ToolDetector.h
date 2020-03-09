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
      ToolDetector(ToolBox& toolBox, const PTopPin& pin)
        : m_toolBox(toolBox)
        , m_pin(pin)
      {
      }

      bool enabled() const { return m_enabled; }
      void enable(bool enable) { m_enabled = enable; }

      bool voltageLoggingEnabled() const { return m_stabilizer.voltageLoggingEnabled(); }
      void enableVoltageLogging(bool enable) { m_stabilizer.enableVoltageLogging(enable); }

      inline void frequentInterruptibleWork();

    private:
      ToolBox& m_toolBox;
      const PTopPin& m_pin;
      volatile bool m_enabled = true;
      unsigned long m_unstableStartTime = 0;

      unsigned long m_nextCheckAt = 0;
      VoltageTypeStabilizer m_stabilizer;

      inline Tool* mapToTool(VoltageType type);
      inline void checkForInstability(unsigned long time);
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

  void ToolDetector::checkForInstability(unsigned long time) {
    // Report if unstable for longer than expected
    if (m_unstableStartTime == 0) {
      auto duration = m_stabilizer.currentInstabliltyDuration();
      if (
        duration < 1000 ||

        // Don't report voltage warnings if the probe is mounted
        // NOTE: Voltage reading are unstable when the probe is
        //       triggered, and while retracted from a triggering
        //       position. During probing (repeated intentional
        //       triggering) the instability can last for several
        //       seconds. Reporting that would be too noisy
        //       and there is little value in adding more complexity
        //       to report more detail for the probe
        m_toolBox.probe.attached()
      ) {
        return;
      }

      m_unstableStartTime = m_stabilizer.currentInstabliltyStartTime();
      logWarning
        << F("Voltage type has been unstable for ")
        << duration
        << F("ms, voltages = [")
        << m_stabilizer.voltages()
        << F("]")
        << endl;

    } else if (m_stabilizer.isStable()) {
      // Instability finished, report additional info
      auto duration = m_unstableStartTime - time;
      log
        << F("Voltage type stabilized after ")
        << duration
        << F("ms, voltages = [")
        << m_stabilizer.voltages()
        << F("]")
        << endl;
      m_unstableStartTime = 0;
    }
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

    // Update voltage classification
    const auto sample = m_pin.value();
    m_stabilizer.add(sample.startTime, sample.voltage);

    // Report if unstable for longer than expected
    checkForInstability(sample.startTime);

    // Update tool
    const auto detectedTool = mapToTool(m_stabilizer.value());
    if (detectedTool && m_enabled) {
      m_toolBox.setTool(detectedTool);
    }
  }
}
