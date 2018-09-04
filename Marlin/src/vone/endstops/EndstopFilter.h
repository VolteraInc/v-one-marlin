#pragma once

#include "../../utils/ScopedInterruptDisable.h"

class EndstopFilter {
  public:
    inline void ignore(bool ignoreTriggers = true);
    inline bool ignored() const;
    inline bool triggered() const;
    inline void addSample(int value);
    inline void reset();

  private:
    static const unsigned int TriggerCountThreshold = 2;
    volatile unsigned int m_triggerCount = 0;
    volatile bool m_ignored = false;

    bool _triggered() const {
      return m_triggerCount >= TriggerCountThreshold;
    }
};

void EndstopFilter::ignore(bool ignoreTriggers) {
  ScopedInterruptDisable sid;
  m_ignored = ignoreTriggers;
}

bool EndstopFilter::ignored() const {
  ScopedInterruptDisable sid;
  return m_ignored;
}

bool EndstopFilter::triggered() const {
  ScopedInterruptDisable sid;
  return !m_ignored && _triggered();
}

void EndstopFilter::addSample(int value) {
  ScopedInterruptDisable sid;
  if (value) {
    // Increment the counter, but prevent overflow
    if (!_triggered()) {
      ++m_triggerCount;
    }
  } else {
    m_triggerCount = 0;
  }
}

void EndstopFilter::reset() {
  ScopedInterruptDisable sid;
  m_triggerCount = 0;
}
