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

  if (m_voltageLoggingEnabled) {
    auto delta = time - m_instabliltyStartTime;
    log
      << F("Voltage type stabilized to ")
      << toString(m_type)
      << F(" after ")
      << delta
      << F("ms, voltages = [")
      << m_voltages
      << F("]")
      << endl;
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
    m_instabliltyStartTime = time;
    m_stableAndReported = false;
  }
}

// Returns the amount of time needed to treat voltage as stable.
static unsigned long s_stabilityThreshold(VoltageType type) {
  switch (type) {
    // Notes:
    //     1) General: it takes about 60ms to climb from 'Triggered' to 'Not Mounted'
    //       (i.e. the entire range). So, seeing the same type for 50ms is
    //        _plenty_ of time
    //
    //     2) ProbeTriggered: The drop to 0 when the probe is trigger is effectively instantaneous
    //        From there it might fluctuate to other values, that have non-zero
    //        stability thresholds. Still to avoid noisy transitions from
    //        (e.g. when the probe is triggered manually) we will wait for a
    //        few samples before classifying as ProbeTriggered
    //
    //     3) Increased from 50 to 200ms because we have seen probes report
    //        "DrillMounted" when the threshold is 50ms. It may be necessary
    //        increase the value further.
    //
    //     4) There is no strong need to detect tool change extremely fast.
    //        Originally, we wanted to detect detaches quickly for "Drill
    //        Safety", but that logic is flawed since the printer can not
    //        stop the drill if it is detached. Moreover, detach detection
    //        should related to 'NoToolMounted' not these values.
    //
    //     5) If you are unlucky, you can see 60ms of Unknowns
    //        This happens if the reading happen to fall between valid ranges
    //        while climbing toward a stable reading. So to avoid classifying
    //        in that case we want this value slightly larger than the time
    //        it takes to climb (see note 1). We increase it further base on
    //        note 4.
    //
    //     6) NoToolMounted is fairly high on the voltage scale so there
    //        wasn't a strong need to collect multiple samples to disambiguate
    //        BUT we have seen the smart dispenser report 'Not Mounted' voltages
    //        after the e-axis is changed (perhaps due to the impact of the gears).
    //        So, we now use a non-zero threshold for NoToolMounted too, and it might
    //        as well use the same value as the other voltage classes.
    //
    case VoltageType::ProbeTriggered:
    case VoltageType::ProbeMounted:
    case VoltageType::SmartDispenserMounted:
    case VoltageType::DrillMounted:
    case VoltageType::Unknown:
    case VoltageType::NoToolMounted:
    default:
      return 200;
  }
}

// How much to delay before reporting stability
static unsigned long s_reportingDelay(VoltageType type) {
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
  if (m_instabliltyStartTime == 0) {
    m_instabliltyStartTime = time;
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

  // Check for stability
  const auto period = m_voltages.timeSpanOfCurrentType();
  if (!m_stable) {
    const auto stabilityThreshold = s_stabilityThreshold(type);
    if (period >= stabilityThreshold) {
      setStable(true, time);
    }
  }

  // Report, if stable, and past delay
  if (m_stable) {
    const auto reportingDelay = s_reportingDelay(type);
    if (period >= reportingDelay) {
      reportStable(time);
    }
  }
}
