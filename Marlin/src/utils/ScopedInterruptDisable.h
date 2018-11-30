#pragma once

#include "../../MarlinConfig.h"

struct ScopedInterruptDisable {
  FORCE_INLINE ScopedInterruptDisable() : sreg(SREG) { noInterrupts(); }
  FORCE_INLINE ~ScopedInterruptDisable() { SREG = sreg; }
private:
  unsigned char sreg;
};
