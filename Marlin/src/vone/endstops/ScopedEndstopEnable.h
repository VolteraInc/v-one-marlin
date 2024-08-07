#pragma once

#include "EndstopMonitor.h"

struct ScopedEndstopEnable {
  FORCE_INLINE ScopedEndstopEnable(
    EndstopMonitor& monitor,
    const Endstop& e1
  )
    : monitor(monitor)
    , m_e1(monitor.ignoring(e1) ? (monitor.ignore(e1, false), &e1) : nullptr)
  {    
    #ifdef XYZ_STRAIN
    if ((m_e1 != nullptr) && (m_e1->virtualEndstop)) {
      monitor.enableXYZ();
      m_virtEs = true;
    }
    #endif
  }

  FORCE_INLINE ~ScopedEndstopEnable() {
    if (m_e1) 
    { 
      monitor.ignore(*m_e1);
      #ifdef XYZ_STRAIN
      if (m_virtEs) {monitor.disableXYZ();}
      #endif
    }
  }
private:
  EndstopMonitor& monitor;
  const Endstop* m_e1;
  bool m_virtEs = false;
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
  { 
    #ifdef XYZ_STRAIN
    if ((m_e1 != nullptr) && (m_e1->virtualEndstop)) {
      monitor.disableXYZ();
      m_virtEs = true;
    }
    #endif
  }

  // Convenience constructor, if the given endstop is null, this is a no-op
  FORCE_INLINE ScopedEndstop_DISABLE(
    EndstopMonitor& monitor,
    const Endstop* e1
  )
    : monitor(monitor)
    , m_e1(!e1 || monitor.ignoring(*e1) ?  nullptr : (monitor.ignore(*e1), e1))
  { 
    #ifdef XYZ_STRAIN
    if ((m_e1 != nullptr) && (m_e1->virtualEndstop)) {
      monitor.disableXYZ();
      m_virtEs = true;
    }
    #endif
  }

  FORCE_INLINE ~ScopedEndstop_DISABLE() {
    if (m_e1)
    { 
      monitor.ignore(*m_e1, false); 
      #ifdef XYZ_STRAIN
      if (m_virtEs) {monitor.enableXYZ();}
      #endif
    }
  }
private:
  EndstopMonitor& monitor;
  const Endstop* m_e1;
  bool m_virtEs = false;
};