#pragma once

#include "../../utils/ScopedInterruptDisable.h"

class EndstopFilter {
  public:
    inline void ignore(bool ignoreTriggers = true);
    inline bool ignored() const;
    inline bool triggered() const;
    inline void addSample(bool value);
    inline void reset();

  private:
    static const int TriggerCountThreshold = 2;
    volatile int m_triggerCount = 0;
    volatile bool m_ignoreTriggers = false;

    bool _triggered() const {
      return m_triggerCount >= TriggerCountThreshold;
    }
};

void EndstopFilter::ignore(bool ignoreTriggers) {
  ScopedInterruptDisable sid;
  m_ignoreTriggers = ignoreTriggers;
}

bool EndstopFilter::ignored() const {
  ScopedInterruptDisable sid;
  return m_ignoreTriggers;
}

bool EndstopFilter::triggered() const {
  ScopedInterruptDisable sid;
  return !m_ignoreTriggers && _triggered();
}

void EndstopFilter::addSample(bool value) {
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
