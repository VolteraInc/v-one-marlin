#pragma once
#include <Arduino.h>
#include "../../../../Marlin.h"
#include "../../../../macros.h"
#include "../../../utils/rawToVoltage.h"
#include "../adc/SamplingHelper.h"

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

    PTopPin(int digitalPin, int analogPin);

    FORCE_INLINE int analogPin() { return _analogPin; }

    FORCE_INLINE Sample voltage();
    FORCE_INLINE Sample readVoltage();

    FORCE_INLINE int readDigitalValue(bool& value);

    int resetTool();

    int send(char* msg);

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

    static const char* modeToString(Mode mode);

    FORCE_INLINE void setMode_Idle();
    FORCE_INLINE bool tryToSetMode_AdcSampling();
    FORCE_INLINE bool trytoSetMode_DirectRead();
    bool trytoSetMode_Communication();
    bool trytoSetMode_ResetTool();

    // ADC sampling
    volatile unsigned long preventAdcSamplingUntil = 0;
    const unsigned numSamples = 4; //  TODO: use 1, 2, 16 ????
    adc::SamplingHelper adcSamples;

    // Direct read
    FORCE_INLINE bool _readDigitalValue();

    // Communication
//     CustomSerial serial;
    long commandSentTime = 0;
};

// ----------------------------------------------
// Idle
// Note: Inlined for use below

void PTopPin::setMode_Idle() {
  ScopedInterruptDisable sid;
  if (mode != Mode::Idle) {
    digitalWrite(_digitalPin, 1);
    mode = Mode::Idle;
  }
}

// ----------------------------------------------
// ADC sampling
// Note: Inline implemtations for use by AnalogDigitalConverter

bool PTopPin::tryToSetMode_AdcSampling() {
  ScopedInterruptDisable sid;
  if (mode == Mode::AdcSamplng) {
    return true;
  }

  if (
    mode != Mode::Idle ||
    millis() < preventAdcSamplingUntil
  ) {
    return false;
  }

  mode = Mode::AdcSamplng;
  pinMode(_digitalPin, INPUT);
  adcSamples.reset();
  return true;
}

bool PTopPin::tryStartAdcSampling() {
  return tryToSetMode_AdcSampling();
}

void PTopPin::addAdcSample(unsigned long value) {
  ScopedInterruptDisable sid;
  if (mode != Mode::AdcSamplng) {
    return;
  }

  // TODO: still need to detect detaches. Perhaps separate from
  // voltage sampling or maybe not.

  auto ready = adcSamples.add(value);
  if (ready) {
    // Delay collection of next value so that we don't
    // stay in INPUT mode too long and reset the router
    // Note: don't wait too long though, or it will slow
    // down tool type classification
    preventAdcSamplingUntil = millis() + 50;

    setMode_Idle();
  }
}

// ----------------------------------------------
// Direct Read
// Note: Inlined for use my stepper. The stepper only needs digital reads, but
//       I'd rather keep all the reads together.

bool PTopPin::trytoSetMode_DirectRead() {
  ScopedInterruptDisable sid;
  if (mode == Mode::Idle) {
    mode = Mode::DirectRead;
    pinMode(_digitalPin, INPUT);
    return true;
  }
  return false;
}

bool PTopPin::_readDigitalValue() {
  // p-top needs to be inverted
  // Note: p-top goes LOW when it triggers and is HIGH when not triggered
  return digitalRead(_digitalPin) ^ 1;
}

int PTopPin::readDigitalValue(bool& value) {
  ScopedInterruptDisable sid;
  if (mode == Mode::AdcSamplng) {
    // AdcSampling mode implies that the pin is already configured for INPUT
    // so we can read and return. It's important not to change the pin mode
    // (to OUTPUT), which would happen if we set the state to IDLE
    value = _readDigitalValue();
    return 0;

  } else if (trytoSetMode_DirectRead()) {
    // Mode was Idle, not it's DirectRead, which sets the pin mode to INPUT.
    // Return the mode to Idle before returning
    value = _readDigitalValue();
    setMode_Idle();
    return 0;

  } else {
    // Digital reads while in other modes would break things (e.g. Communication).
    // Preventing this is outside the scope of this class.
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to read digital value of p-top pin, ");
    SERIAL_ERROR(modeToString(mode));
    SERIAL_ERRORPGM(" mode does not support digital reads");
    SERIAL_EOL;
    return -1;
  }
}

PTopPin::Sample PTopPin::voltage()     { return Sample( adcSamples.value()     ); }
PTopPin::Sample PTopPin::readVoltage() { return Sample( adcSamples.readValue() ); }
