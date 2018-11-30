#pragma once

#include "../../serial.h"

class __FlashStringHelper;

class Reporter {
  public:
    Reporter(
      const __FlashStringHelper* prefix,
      const __FlashStringHelper* name,
      const __FlashStringHelper* units,
      unsigned long initialValue = 0
    )
      : m_value(initialValue)
      , m_prefix(prefix)
      , m_name(name)
      , m_units(units)
      , m_reportedValue(initialValue)
    {
    }

    void outputStatus() {
      log
        << m_prefix
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

  protected:
    volatile unsigned long m_value;

  private:
    const __FlashStringHelper* m_prefix;
    const __FlashStringHelper* m_name;
    const __FlashStringHelper* m_units;
    unsigned long m_reportedValue;
};

class HighWaterReporter: public Reporter {
  inline static const __FlashStringHelper* s_prefix() { return F("High-water mark for "); }

  public:
    HighWaterReporter(
      const __FlashStringHelper* name,
      const __FlashStringHelper* units,
      unsigned long initialValue = 0
    ): Reporter(s_prefix(), name, units, initialValue) {
    }

    FORCE_INLINE void updateIfHigher(unsigned long newValue) {
      if (newValue > m_value) {
        m_value = newValue;
      }
    }
};

class LowWaterReporter: public Reporter {
  inline static const __FlashStringHelper* s_prefix() { return F("Low-water mark for "); }

  public:
    LowWaterReporter(
      const __FlashStringHelper* name,
      const __FlashStringHelper* units,
      unsigned long initialValue = 0
    ): Reporter(s_prefix(), name, units, initialValue) {
    }

    FORCE_INLINE void updateIfLower(unsigned long newValue) {
      if (newValue < m_value) {
        m_value = newValue;
      }
    }
};