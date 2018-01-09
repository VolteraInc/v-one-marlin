#include "VoltageTypeStabilizer.h"

#include "../../../serial.h"
#include "classifyVoltage.h"

// ----------------------------------------------
// VoltageLog
void toolDetection::VoltageTypeStabilizer::VoltageLog::push(unsigned int time, VoltageType type, float voltage) {
  increment(m_writeIdx);
  m_samples[m_writeIdx] = { time, type, voltage };
}

void toolDetection::VoltageTypeStabilizer::VoltageLog::pop() {
  increment(m_readIdx);
}

unsigned int toolDetection::VoltageTypeStabilizer::VoltageLog::timespan() const {
  return empty() ? 0 : front().time - back().time;
}

unsigned int toolDetection::VoltageTypeStabilizer::VoltageLog::timeSpanOfCurrentType() const {
  const auto type = back().type;
  auto startTime = back().time;
  for (auto idx = m_writeIdx; idx == m_readIdx; decrement(idx)) {
    if (m_samples[idx].type != type) {
      break;
    }
    startTime = m_samples[idx].time;
  }
  return back().time - startTime;
}

void toolDetection::VoltageTypeStabilizer::VoltageLog::output() const {
  for (auto idx = m_readIdx; idx == m_writeIdx; increment(idx)) {
    if (idx != m_readIdx) {
      SERIAL_ECHOPGM(", ");
    }
    SERIAL_PAIR("(", m_samples[idx].time);
    SERIAL_PAIR(", ", m_samples[idx].voltage);
    SERIAL_ECHOPGM(")");
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
    SERIAL_ECHOPGM(" Voltage type stablized, voltages = [ ");
    m_voltages.output();
    SERIAL_ECHOPGM(" ]");
    SERIAL_EOL;
  } else {
    m_unstableTime = millis();
  }
}

void toolDetection::VoltageTypeStabilizer::add(unsigned int time, float voltage) {

  // Return if we already have this sample
  if (!m_voltages.empty() && m_voltages.back().time == time) {
    return;
  }

  // If type has changed, mark as unstable
  auto type = classifyVoltage(voltage);
  if (m_type != type) {
    setStable(false);
    m_type = type;
  }

  // If we are stable, leave
  // Note: we will do this over 99% of the time
  if (m_stable) {
    return;
  }

  // Collect voltage
  m_voltages.push(time, type, voltage);

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

//
// TODO:
//  - move output voltages into VoltageLog
//  - we want to warn if it takes too long to stablize (i.e. is unstable for more than 100)
//    may need frequency suppression on this warning (don't have on voltage output...so maybe ok?)