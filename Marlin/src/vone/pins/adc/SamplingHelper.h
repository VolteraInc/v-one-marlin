#pragma once

#include <Arduino.h>
#include "../../../../macros.h"
#include "../../../../Marlin.h"
#include "SampledValue.h"

namespace adc {

class SamplingHelper {
  private:
    inline void updateValue() {
      // swap pointers
      if (writePtr == &bufferA) {
        writePtr = &bufferB;
        readPtr = &bufferA;
      } else {
        writePtr = &bufferA;
        readPtr = &bufferB;
      }

      // reset write buffer for next set of samples
      writePtr->reset();
    }

  public:
    inline SamplingHelper(int numSamples)
      : bufferA(numSamples)
      , bufferB(numSamples)
    {
    }

    inline bool add(long value) {
      ScopedCriticalSection scs;
      const auto ready = writePtr->add(value);
      if (ready) {
        updateValue();
      }
      return ready;
    }

    inline void reset() {
      ScopedCriticalSection scs;
      writePtr->reset();
    }

    inline long value() {
      ScopedCriticalSection scs;
      return readPtr->value();
    }

    inline long readValue() {
      const auto* tmp = readPtr;
      while (tmp == readPtr) {
      }
      return value();
    }

  private:
    SampledValue bufferA;
    SampledValue bufferB;
    SampledValue* writePtr = &bufferA;
    const SampledValue* volatile readPtr = &bufferB;
};

}
