#pragma once
#include "../adc/SamplingHelper.h"

class BedTemperaturePin {
  public:
    BedTemperaturePin(int analogPin);

    int analogPin() const { return _analogPin; }

    inline float temperature();     // same as value(), but converts to voltage
    inline float readTemperature(); // same as readValue(), but converts to voltage

    // ADC sampling
    inline void addAdcSample(unsigned long value) __attribute__((always_inline));

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

float BedTemperaturePin::temperature()     { return rawToTemperature( adcSamples.value()     ); }
float BedTemperaturePin::readTemperature() { return rawToTemperature( adcSamples.readValue() ); }
