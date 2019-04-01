#pragma once

#include "../../../macros.h"
#include "../../utils/ScopedInterruptDisable.h"

class EndstopFilter {
  public:
    FORCE_INLINE void ignore(bool ignoreTriggers = true);
    FORCE_INLINE bool ignored() const;
    FORCE_INLINE bool triggered() const;
    FORCE_INLINE void addSample(int value);
    FORCE_INLINE void reset();

  private:
    static const unsigned int TriggerCountThreshold = 2;
    volatile unsigned int m_triggerCount = 0;
    volatile bool m_ignored = false;

    FORCE_INLINE bool _triggered() const {
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
    // Note: Don't increment the counter if already
    //       triggered, this prevents integer overflow
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
