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

// Note: Disabling switches should not be overlooked
//       (especially given the similarity to the enable class above)
//       hence the _DISABLE suffix
struct ScopedEndstop_DISABLE {
  FORCE_INLINE ScopedEndstop_DISABLE(
    EndstopMonitor& monitor,
    const Endstop& e1
  )
    : monitor(monitor)
    , m_e1(monitor.ignoring(e1) ?  nullptr : (monitor.ignore(e1), &e1))
  { }

  // Convenience constructor, if the given endstop is null, this is a no-op
  FORCE_INLINE ScopedEndstop_DISABLE(
    EndstopMonitor& monitor,
    const Endstop* e1
  )
    : monitor(monitor)
    , m_e1(!e1 || monitor.ignoring(*e1) ?  nullptr : (monitor.ignore(*e1), e1))
  { }

  FORCE_INLINE ~ScopedEndstop_DISABLE() {
    if (m_e1) { monitor.ignore(*m_e1, false); }
  }
private:
  EndstopMonitor& monitor;
  const Endstop* m_e1;
};

//added to control the XYZ mode for endstop monitor
struct XYZModeEnable {
  FORCE_INLINE XYZModeEnable(
    EndstopMonitor& monitor
  )
    : monitor(monitor)
  {
    monitor.enableXYZ();
  }

  FORCE_INLINE ~XYZModeEnable() {
    monitor.disableXYZ();
  }
private:
  EndstopMonitor& monitor;
};