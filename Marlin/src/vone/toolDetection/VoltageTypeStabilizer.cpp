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

void toolDetection::VoltageTypeStabilizer::reportStable(unsigned long time) {
  if (!m_stable) {
    logWarning << "Attempted to report stability before readings stabilized" << endl;
    return;
  }

  m_stableAndReported = true;
  auto delta = time - m_unstableTime;

  log
    << F("Voltage type stabilized in ") << delta
    << F("ms, voltages = [")
    << m_voltages
    << F("]")
    << endl;

  // Warn if unstable for longer than expected
  if (delta > 1000) {
    logWarning << F("Voltage type was unstable for ") << delta << F("ms") << endl;
  }
}

void toolDetection::VoltageTypeStabilizer::setStable(bool stable, unsigned long time) {
  if (m_stable == stable) {
    return;
  }

  m_stable = stable;

  // Record time when we become unstable
  // Note: we do this so we can report lengthy periods of instability
  if (!m_stable) {
    m_unstableTime = time;
    m_stableAndReported = false;
  }
}

// Returns the amount of time needed to treat voltage as stable.
static unsigned long s_stabilityThreshold(VoltageType type) {
  switch (type) {
    // NoToolMounted is already high on the voltage scale
    // no need to collect multiple samples to disambiguate
    case VoltageType::NoToolMounted:
      return 0;

    // The drop to 0 when the probe is trigger is effectively instantaneous
    // For there it might fluctuate to other values, that have non-zero
    // stability thresholds.
    case VoltageType::ProbeTriggered:
      return 0;

    // Note: it takes about 60ms to climb from 'Triggered' to 'Not Mounted'
    //       (i.e. the entire range). So, seeing the same type for 50ms is
    //        _plenty_ of time
    case VoltageType::ProbeMounted:
    case VoltageType::SmartDispenserMounted:
    case VoltageType::DrillMounted:
    case VoltageType::DrillResetting:
    case VoltageType::Unknown:
    default:
      return 50;
  }
}

// How much to delay before reporting stability
static unsigned long s_reportingThreshold(VoltageType type) {
  switch (type) {
    // Delay reporting triggered so we don't generate nuisance
    // messages while probing. But we will report if the probe is
    // triggered for much longer than expected.
    case VoltageType::ProbeTriggered:
      return 200;

    // In most cases we want no delay
    default:
      return 0;
  }
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
    setStable(false, time);
    m_type = type;
  }

  // If we are stable (and have it reported), leave
  // Note: we will do this over 99% of the time
  if (m_stableAndReported) {
    return;
  }

  // Check for stability and reporting threshold
  const auto period = m_voltages.timeSpanOfCurrentType();
  if (!m_stable) {
    const auto stabilityThreshold = s_stabilityThreshold(type);
    if (period >= stabilityThreshold) {
      setStable(true, time);
    }
  }
  const auto reportingThreshold = s_reportingThreshold(type);
  if (period >= reportingThreshold) {
    reportStable(time);
  }
}
