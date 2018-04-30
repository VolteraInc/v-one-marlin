#pragma once
#include "../../../../MarlinConfig.h"
#include "../../../../serial.h"
#include "../../../utils/rawToVoltage.h"
#include "../adc/SamplingHelper.h"

#include <SoftwareSerial.h>

class PTopPin {
  public:
    struct Sample {
      float voltage = 0;
      unsigned long startTime = 0;
      unsigned long endTime = 0;

      Sample() {}
      Sample(const adc::SampledValue& adcSample)
        : voltage(rawToVoltage(adcSample.value()))
        , startTime(adcSample.startTime)
        , endTime(adcSample.endTime)
      {
      }
    };

    inline PTopPin(int digitalPin, int analogPin);

    FORCE_INLINE int analogPin() const { return _analogPin; }

    // Returns the most recently sampled value
    FORCE_INLINE Sample value() const;

    // Blocks until a new sample is available, then returns it
    FORCE_INLINE Sample readValue();

    FORCE_INLINE int readDigitalValue(bool& value);

    inline int resetTool();

    inline int send(char* msg);

    // ADC sampling
    FORCE_INLINE bool tryStartAdcSampling();
    FORCE_INLINE void addAdcSample(unsigned long value);

  private:
    int _digitalPin;
    int _analogPin;

    // Modes
    enum class Mode: int {
      Idle,
      AdcSamplng,
      Communication,
      DirectRead,
      ResetTool
    };
    volatile Mode mode = Mode::Idle;

    static const char* modeToString(Mode mode) {
      switch(mode) {
        case Mode::Idle: return "Idle";
        case Mode::Communication: return "Communication";
        case Mode::ResetTool: return "Reset Tool";
        case Mode::AdcSamplng: return "Analog Sampling";
        case Mode::DirectRead: return "Direct Read";
      }
      return "Unknown";
    };

    FORCE_INLINE void setMode_Idle();
    FORCE_INLINE bool tryToSetMode_AdcSampling();
    FORCE_INLINE bool trytoSetMode_DirectRead();
    inline bool trytoSetMode_Communication();
    inline bool trytoSetMode_ResetTool();

    // ADC sampling
    // Note: our adc readings seem stable, plus averaging
    // is a horrible way to stabilize. Also, it's best to
    // address instability at a higher level of the code
    // where there is more context (e.g. analog probing,
    // heater control, etc).
    const unsigned numSamples = 1; // see note^
    volatile bool preventAdcSampling = false;
    volatile unsigned long delayAdcSamplingUntil = 0;
    unsigned long lastAdcSampleStartedAt = 0;
    adc::SamplingHelper adcSamples;

    // We want the default value to classify as VoltageType::NoToolMounted
    // This means we want rawToVoltage(DefaultRawVoltage) == 5.0,
    // which works out to 1024.0f
    static constexpr float DefaultRawVoltage = 1024.0f;

    // Direct read
    FORCE_INLINE bool _readDigitalValue();


    // Communication

    // baudrate of 300 is based on the rise and fall times of the capacitor on ptop (600-800 us)
    const int baud = 300;
    const int dummy_pin = A3;

    SoftwareSerial serial;
    long timeSent = 0;

    inline void _sendMessage(char* msg);
    inline int _recvAcknowledgement();
};


PTopPin::PTopPin(int digitalPin, int analogPin)
  : _digitalPin(digitalPin)
  , _analogPin(analogPin)
  , adcSamples(numSamples, DefaultRawVoltage)
  , serial(dummy_pin, _digitalPin)
{
  // Reset attached tool, if there is one
  resetTool();
}

PTopPin::Sample PTopPin::value() const { return Sample( adcSamples.value()     ); }
PTopPin::Sample PTopPin::readValue()   { return Sample( adcSamples.readValue() ); }

// ----------------------------------------------
// Idle

void PTopPin::setMode_Idle() {
  ScopedInterruptDisable sid;
  if (mode != Mode::Idle) {
    digitalWrite(_digitalPin, HIGH);
    if (mode != Mode::AdcSamplng) {
      preventAdcSampling = false;
    }
    mode = Mode::Idle;
  }
}
