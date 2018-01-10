#include "VoltageTypeStabilizer.h"

#include "../../../serial.h"
#include "classifyVoltage.h"

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

void toolDetection::VoltageTypeStabilizer::VoltageLog::output() const {
  if (empty()) {
    return;
  }
  auto idx = m_frontIdx;
  auto sz = size();
  while (sz--) {
    if (idx != m_frontIdx) {
      SERIAL_ECHOPGM(", ");
    }
    SERIAL_PAIR("(", m_samples[idx].time);
    SERIAL_PAIR(", ", toString(m_samples[idx].type));
    SERIAL_PAIR(", ", m_samples[idx].voltage);
    SERIAL_ECHOPGM(")");
    idx = increment(idx);
  }
}

// ----------------------------------------------
// VoltageTypeStabilizer
void toolDetection::VoltageTypeStabilizer::setStable(bool stable) {
  if (m_stable == stable) {
    return;
  }
  m_stable = stable;

  // Log voltages when we stablized
  if (m_stable) {
    auto delta = millis() - m_unstableTime;
    if (delta > 1000) {
      SERIAL_ECHO_START;
      SERIAL_PAIR("WARNING: Voltage type was unstable for ", delta);
      SERIAL_ECHOPGM("ms");
      SERIAL_EOL;
    }

    SERIAL_ECHO_START;
    SERIAL_PAIR("Voltage type stablized in ", delta);
    SERIAL_ECHOPGM("ms, voltages = [ ");
    m_voltages.output();
    SERIAL_ECHOPGM(" ]");
    SERIAL_EOL;
  } else {
    m_unstableTime = millis();
  }
}

void toolDetection::VoltageTypeStabilizer::add(unsigned long time, float voltage) {
  // Return if we already have this sample
  if (!m_voltages.empty() && m_voltages.back().time == time) {
    return;
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

  if (
    // NoToolMounted is already high on the voltage scale
    // no need to collect multiple samples to disambiguate
    m_type == VoltageType::NoToolMounted ||

    // Note: it takes about 60ms to climb from triggered to not mounted
    //       so, 40ms without changing type is plenty of time.
    m_voltages.timeSpanOfCurrentType() >= 40
  ) {
    setStable(true);
  }
}
