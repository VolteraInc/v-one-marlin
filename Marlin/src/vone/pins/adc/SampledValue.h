#pragma once

#include <Arduino.h>

namespace adc {

struct SampledValue {
  unsigned _numSamples;
  unsigned count = 0;
  unsigned long sum = 0;
  unsigned long startTime = 0;
  unsigned long endTime = 0;

  FORCE_INLINE SampledValue(unsigned numSamples)
    : _numSamples(numSamples)
  {
  }

  FORCE_INLINE float value() const volatile {
    return sum / count;
  };

  FORCE_INLINE void reset() {
    count = 0;
    sum = 0;
    startTime = 0;
    endTime = 0;
  }

  FORCE_INLINE bool add(unsigned long value) {
    if (count == 0) {
      startTime = millis();
    }
    ++count;
    sum += value;
    if (count >= _numSamples) {
      endTime = millis();
      return true;
    }
    return false;
  }
};

}
