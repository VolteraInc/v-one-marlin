#pragma once

#include "BedTemperaturePin.h"
#include "ThermistorTable.h"
#include "../adc/SamplingHelper.h"

#include "../../../../Marlin.h"

// Note: we use 16 because Marlin did
#define NUM_SAMPLES_BED 16

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

    inline BedTemperaturePin(int analogPin);

    int analogPin() const { return _analogPin; }

    // Returns the most recently sampled value
    FORCE_INLINE Sample value();

    // Blocks until a new sample is available, then returns it
    FORCE_INLINE Sample readValue();

    // ADC sampling
    FORCE_INLINE void addAdcSample(unsigned long value);

  private:
    int _analogPin;

    // ADC sampling
    const unsigned numSamples = NUM_SAMPLES_BED;
    adc::SamplingHelper adcSamples;

    static inline float rawToTemperature(long raw);
};


void BedTemperaturePin::addAdcSample(unsigned long value) { adcSamples.add(value); }

BedTemperaturePin::Sample BedTemperaturePin::value()     { return Sample(adcSamples.value()     ); }
BedTemperaturePin::Sample BedTemperaturePin::readValue() { return Sample(adcSamples.readValue() ); }

BedTemperaturePin::BedTemperaturePin(int analogPin)
  : _analogPin(analogPin)
  , adcSamples(numSamples, DEFAULT_VOLTAGE)
{
  // Disable the digital read buffer on this pin
  // Note: We only do analog reads on TEMP_BED_PIN, so we can disable its
  // digital read buffer.
  //   "If you are doing only ADC conversions on some or all of the analog
  //    inputs, you should disable the digital buffers, to save power.
  //    Once disabled, a digitalRead on those pins will always read zero."
  //       - https://www.gammon.com.au/adc
  log << F("Disable digital reads of the bed temperature pin") << endl;
  SBI(DIDR0, _analogPin);
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)

// DEFER: this code could be significantly clearer
// could potentially move to utils too
float BedTemperaturePin::rawToTemperature(long raw) {
  float celsius;
  const auto size = sizeof(temperatureTable)/sizeof(*temperatureTable);
  for (auto i = 1u; i < size; ++i) {
    if (PGM_RD_W(temperatureTable[i][0]) > raw) {
      celsius = (
        PGM_RD_W(temperatureTable[i-1][1]) +
          (raw - PGM_RD_W(temperatureTable[i-1][0])) *
          (float)(PGM_RD_W(temperatureTable[i][1]) - PGM_RD_W(temperatureTable[i-1][1])) /
          (float)(PGM_RD_W(temperatureTable[i][0]) - PGM_RD_W(temperatureTable[i-1][0]))
      );
      return celsius;
    }
  }

  // Overflow, use last value in table
  celsius = PGM_RD_W(temperatureTable[size - 1][1]);
  return celsius;
}
