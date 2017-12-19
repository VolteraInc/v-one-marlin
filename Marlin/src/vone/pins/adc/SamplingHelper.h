#pragma once

#include <Arduino.h>
#include "../../../../macros.h"
#include "../../../../Marlin.h"
#include "../../../utils/ScopedInterruptDisable.h"
#include "SampledValue.h"

namespace adc {

class SamplingHelper {
  private:
    FORCE_INLINE void updateValue() {
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
    FORCE_INLINE SamplingHelper(int numSamples, float defaultValue)
      : bufferA(numSamples, defaultValue)
      , bufferB(numSamples, defaultValue)
    {
    }

    FORCE_INLINE bool add(long value) {
      ScopedInterruptDisable sid;
      const auto ready = writePtr->add(value);
      if (ready) {
        updateValue();
      }
      return ready;
    }

    FORCE_INLINE void reset() {
      ScopedInterruptDisable sid;
      writePtr->reset();
    }

    FORCE_INLINE SampledValue value() {
      ScopedInterruptDisable sid;
      return *readPtr;
    }

    FORCE_INLINE SampledValue readValue() {
      const auto* tmp = readPtr;
      while (tmp == readPtr) {
        //TODO: make a niceDelay function that logs if it blocks for more than 1s
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
