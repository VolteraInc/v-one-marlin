#include "VoltageTypeStabilizer.h"

#include "../../../serial.h"
#include "classifyVoltage.h"

using namespace toolDetection;
// ----------------------------------------------
// VoltageLog
void toolDetection::VoltageTypeStabilizer::VoltageLog::push(unsigned long time, VoltageType type, float voltage) {
  // Advance front and back
  // Note: if we are empty the front and back already point where we need them to be.
  if (!empty()) {
    if (full()) {
      pop();
    }
    m_backIdx = increment(m_backIdx);
  }

  // store sample
  m_samples[m_backIdx] = { time, type, voltage };
  ++m_size;
}

void toolDetection::VoltageTypeStabilizer::VoltageLog::pop() {
  if (!empty()) {
    m_frontIdx = increment(m_frontIdx);
    --m_size;
  }
}

unsigned long toolDetection::VoltageTypeStabilizer::VoltageLog::timespan() const {
  return front().time - back().time;
}

unsigned long toolDetection::VoltageTypeStabilizer::VoltageLog::timeSpanOfCurrentType() const {
  if (empty()) {
    return 0;
  }
  const auto type = back().type;
  auto startTime = back().time;
  auto idx = m_backIdx;
  auto sz = size();
  while (sz--) {
    if (m_samples[idx].type != type) {
      break;
    }
    startTime = m_samples[idx].time;
    idx = decrement(idx);
  }
  return back().time - startTime;
}

// ----------------------------------------------
// VoltageTypeStabilizer
void toolDetection::VoltageTypeStabilizer::setStable(bool stable) {
  if (m_stable == stable) {
    return;
  }
  m_stable = stable;

  // Log voltages when we stabilized
  if (m_stable) {
    auto delta = millis() - m_unstableTime;
    if (delta > 1000) {
      logWarning << F("Voltage type was unstable for ") << delta << F("ms") << endl;
    }

    log
      << F("Voltage type stabilized in ") << delta
      << F("ms, voltages = [ ")
      << m_voltages
      << F(" ]")
      << endl;
  } else {
    m_unstableTime = millis();
  }
}

// Returns the amount of time needed to treat voltage as stable.
static unsigned long s_stabilityThreshold(VoltageType type) {
  switch (type) {
    // NoToolMounted is already high on the voltage scale
    // no need to collect multiple samples to disambiguate
    case VoltageType::NoToolMounted:
      return 0;

    // Use a longer duration for transitory voltage types
    // Note: this should, in effect, hide these types, unless
    //       something has gone wrong as the voltage is stuck
    case VoltageType::ProbeTriggered:
    case VoltageType::DrillResetting:
    case VoltageType::Unknown:
      return 200;

    // Note: it takes about 60ms to climb from 'Triggered' to 'Not Mounted'
    //       (i.e. the entire range). So, seeing the same type for 30ms is
    //        _plenty_ of time
    case VoltageType::ProbeMounted:
    case VoltageType::DrillMounted:
    default:
      return 50;
  }

// 617772,Probe Mounted, 3.471680
// 617790,Probe Mounted, 3.471680
// 617812,Probe Mounted, 3.471680
// 617831,Probe Mounted, 3.471680
// 617851,Probe Mounted, 3.476562
// 617872,Probe Mounted, 3.471680
// 617892,Probe Mounted, 3.471680
// 617911,Probe Mounted, 3.471680
// 617931,Probe Mounted, 3.471680
// 617952,Probe Mounted, 3.471680
// 617972,Probe Mounted, 3.471680
// 617991,Drill Resetting, 0.336914
// 618011,Unknown, 0.131836
// 618032,Triggered, 0.000000
// 618052,Unknown, 0.092773
// 618071,Unknown, 0.708008
// 618091,Unknown, 0.800781
// 618112,Drill Mounted, 1.210937
// 618132,Drill Mounted, 1.269531   // I suspect some intermittent contect here
// 618152,Drill Mounted, 1.401367
//  ...prob unknowns...
// 618210,Probe Mounted, 3.291016
// 618230,Probe Mounted, 3.393555
// 618251,Probe Mounted, 3.437500

// TODO: 1) move voltage classification to tools ... then we shouldn't see.
//          unless we miss the detach, and the new tool is suddenly present
//       2) increase timespan for now?
}

void toolDetection::VoltageTypeStabilizer::add(unsigned long time, float voltage) {
  // Return if we already have this sample
  if (!m_voltages.empty() && m_voltages.back().time == time) {
    return;
  }

  // Initialize unstable time based on first sample
  if (m_unstableTime == 0) {
    m_unstableTime = time;
  }

  // Collect sample
  auto type = classifyVoltage(voltage);
  m_voltages.push(time, type, voltage);

  // If type has changed, mark as unstable
  if (m_type != type) {
    setStable(false);
    m_type = type;
  }

  // If we are stable, leave
  // Note: we will do this over 99% of the time
  if (m_stable) {
    return;
  }

  // Check for stability
  const auto threshold = s_stabilityThreshold(type);
  if (m_voltages.timeSpanOfCurrentType() >= threshold) {
    setStable(true);
  }
}
