#pragma once

#include "../../utils/ScopedInterruptDisable.h"

class EndstopFilter {
  public:
    inline bool triggered() const;
    inline void addSample(bool value);
    inline void reset();

  private:
    int m_counter = 0;
};


bool EndstopFilter::triggered() const {
  ScopedInterruptDisable sid;
  m_counter >= 2;
}

void EndstopFilter::addSample(bool value) {
  if (value) {
    ++m_counter;
  } else {
    m_counter = 0;
  }
}

void EndstopFilter::reset() {
  ScopedInterruptDisable sid;
  m_counter = 0;
}
