#pragma once
#include "../adc/SamplingHelper.h"

class BedTemperaturePin {
  public:
    struct Sample {
      float temperature = 0;
      unsigned long startTime = 0;
      unsigned long endTime = 0;

      Sample() {}
      Sample(const adc::SampledValue& adcSample)
        : temperature(rawToTemperature(adcSample.value()))
        , startTime(adcSample.startTime)
        , endTime(adcSample.endTime)
      {
      }
    };

    BedTemperaturePin(int analogPin);

    int analogPin() const { return _analogPin; }

    FORCE_INLINE Sample temperature();     // same as value(), but converts to voltage
    FORCE_INLINE Sample readTemperature(); // same as readValue(), but converts to voltage

    // ADC sampling
    FORCE_INLINE void addAdcSample(unsigned long value);

  private:
    int _analogPin;

    // ADC sampling
    const unsigned numSamples = 16; // Note: we use 16 becuase Marlin does
    adc::SamplingHelper adcSamples;

    static float rawToTemperature(long raw);
};


// ----------------------------------------------
// Inline Definitions
// Note: inlined because they are used in ISRs

void BedTemperaturePin::addAdcSample(unsigned long value) { adcSamples.add(value); }

BedTemperaturePin::Sample BedTemperaturePin::temperature()     { return Sample(adcSamples.value()     ); }
BedTemperaturePin::Sample BedTemperaturePin::readTemperature() { return Sample(adcSamples.readValue() ); }
