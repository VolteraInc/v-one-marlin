#pragma once

#include "../../../temperature.h"

class PTopScopedUsageLock {
  bool mustRelease = false;

  // can't copy this class
  PTopScopedUsageLock(PTopScopedUsageLock const&) = delete;
  PTopScopedUsageLock& operator=(PTopScopedUsageLock const&) = delete;

public:
  PTopScopedUsageLock() {
    if (periodicAnalogReadsEnabled()) {
      mustRelease = true;
      set_p_top_mode(P_TOP_COMMS_READ_MODE);
    }
  }

  ~PTopScopedUsageLock() {
    if (mustRelease) {
      set_p_top_mode(P_TOP_NORMAL_MODE);
    }
  }
};
