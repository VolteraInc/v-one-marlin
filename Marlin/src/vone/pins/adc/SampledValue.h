#pragma once

#include <Arduino.h>

namespace adc {

struct SampledValue {
  unsigned _numSamples;
  unsigned count = 0;
  unsigned long sum = 0;
  unsigned long startTime = 0;
  unsigned long endTime = 0;

  inline SampledValue(unsigned numSamples)
    : _numSamples(numSamples)
  {
  }

  inline float value() const volatile {
    return sum / count;
  };

  inline void reset() {
    count = 0;
    sum = 0;
    startTime = 0;
    endTime = 0;
  }

  inline bool add(unsigned long value) {
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
