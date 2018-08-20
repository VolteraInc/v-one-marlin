#pragma once

#include "EndstopMonitor.h"

struct ScopedEndstopEnable {
  FORCE_INLINE ScopedEndstopEnable(
    EndstopMonitor& monitor,
    const Endstop& e1
  )
    : monitor(monitor)
    , m_e1(monitor.ignoring(e1) ? (monitor.ignore(e1, false), &e1) : nullptr)
  {}

  FORCE_INLINE ~ScopedEndstopEnable() {
    if (m_e1) { monitor.ignore(*m_e1); }
  }
private:
  EndstopMonitor& monitor;
  const Endstop* m_e1;
};
