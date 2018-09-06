#pragma once

#include "../../serial.h"

class __FlashStringHelper;

class HighwaterReporter {
  public:
    HighwaterReporter(
      const __FlashStringHelper* name,
      const __FlashStringHelper* units,
      unsigned long initialValue = 0
    )
      : m_name(name)
      , m_units(units)
      , m_value(initialValue)
      , m_reportedValue(initialValue)
    {
    }

    FORCE_INLINE void update(unsigned long newValue) {
      if (newValue > m_value) {
        m_value = newValue;
      }
    }

    FORCE_INLINE void updateIfLower(unsigned long newValue) {
      if (newValue < m_value) {
        m_value = newValue;
      }
    }

    void outputStatus() {
      log
        << F("Highwater mark for ")
        << m_name
        << F(", now at: ")
        << m_value
        << m_units
        << endl;
    }

    void reportIfChanged() {
      if (m_reportedValue != m_value) {
        outputStatus();
        m_reportedValue = m_value;
      }
    }

  private:
    const __FlashStringHelper* m_name;
    const __FlashStringHelper* m_units;
    volatile unsigned long m_value;
    unsigned long m_reportedValue;
};
