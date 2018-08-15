#pragma once

#include "../../../serial.h"
#include "../endstops/Endstop.h"

struct TriggerEvent {
  const Endstop* endstop = nullptr;
  float positionInAxis = 0;

  // default ctor needed so we can have an array (i.e. m_log)
  TriggerEvent() {}

  // needed so we can assign values into m_log[idx]
  TriggerEvent(const Endstop& endstop, float positionInAxis)
    : endstop(&endstop)
    , positionInAxis(positionInAxis)
  {}
};

class TriggerEventLog {
  public:
    inline unsigned int size() const {
      ScopedInterruptDisable sid;
      return m_size;
    }

    inline bool empty() const { return size() == 0; }
    inline bool full() const { return size() == MAX_ENTRIES; }

    inline const TriggerEvent& front() const { return m_log[m_frontIdx]; }
    inline const TriggerEvent& back() const { return m_log[m_backIdx]; }

    inline void push(const Endstop& endstop, float positionInAxis);
    inline void pop();

  private:
    static const unsigned int MAX_ENTRIES = 12;
    TriggerEvent m_log[MAX_ENTRIES];
    volatile unsigned int m_size = 0;
    unsigned int m_frontIdx = 0;
    unsigned int m_backIdx = 0;

    static inline unsigned int increment(unsigned int idx) {
      return idx == MAX_ENTRIES - 1 ? 0 : idx + 1;
    }
};

void TriggerEventLog::push(const Endstop& endstop, float positionInAxis) {
  ScopedInterruptDisable sid;

  // Advance front and back
  // Note: if we are empty the front and back already point where we need them to be.
  if (!empty()) {
    if (full()) {
      // Drop
      logWarning
        << F("Too many trigger events, dropping event for")
        << endstop.name
        << F("at ")
        << axis_codes[endstop.axis]
        << F("=")
        << positionInAxis
        << endl;
      return;
    }
    m_backIdx = increment(m_backIdx);
  }

  // Store event
  m_log[m_backIdx] = { endstop, positionInAxis };
  ++m_size;
}

void TriggerEventLog::pop() {
  ScopedInterruptDisable sid;

  if (!empty()) {
    m_frontIdx = increment(m_frontIdx);
    --m_size;
  }
}
