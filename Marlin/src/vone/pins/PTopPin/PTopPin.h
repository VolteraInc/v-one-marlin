#pragma once
#include <Arduino.h>
#include "../../../../Marlin.h"
#include "../../../../macros.h"
#include "../../../utils/rawToVoltage.h"
#include "../adc/SamplingHelper.h"

#include "CustomSerial.h"

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
    FORCE_INLINE Sample value();

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
    volatile unsigned long preventAdcSamplingUntil = 0;
    const unsigned numSamples = 4; //  TODO: use 1, 2, 16 ????
    adc::SamplingHelper adcSamples;

    // Direct read
    FORCE_INLINE bool _readDigitalValue();

    // Communication
    CustomSerial customSerial;
};

PTopPin::PTopPin(int digitalPin, int analogPin)
  : _digitalPin(digitalPin)
  , _analogPin(analogPin)
  , adcSamples(numSamples)
  , customSerial(digitalPin)
{
  // Reset attached tool, if there is one
  resetTool();
}

// ----------------------------------------------
// Idle
// Note: Inlined for use below

void PTopPin::setMode_Idle() {
  ScopedInterruptDisable sid;
  if (mode != Mode::Idle) {
    digitalWrite(_digitalPin, HIGH);
    mode = Mode::Idle;
  }
}

// ----------------------------------------------
// Reset tool

bool PTopPin::trytoSetMode_ResetTool() {
  ScopedInterruptDisable sid;
  if (mode == Mode::Idle) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Resetting tool");
    digitalWrite(_digitalPin, LOW);
    mode = Mode::ResetTool;
    return true;
  }
  return false;
}

int PTopPin::resetTool() {
  while (trytoSetMode_ResetTool()) {
    delay(1);
  }
  delay(500);
  setMode_Idle();
  return 0;
}


// ----------------------------------------------
// Communication

bool PTopPin::trytoSetMode_Communication() {
  ScopedInterruptDisable sid;
  if (mode == Mode::Idle) {
    mode = Mode::Communication;
    return true;
  }
  return false;
}

int PTopPin::send(char* msg) {
  int returnValue = -1;

  while (!trytoSetMode_Communication()) {
    delay(1);
  }

  const unsigned maxAttempts = 3;
  unsigned numAttempts = 0;
  if (customSerial.send(msg, maxAttempts, &numAttempts)) {
    goto DONE;
  }

  returnValue = 0;

DONE:
  setMode_Idle();

  if (returnValue == 0 && numAttempts > 2) {
    SERIAL_ECHO_START;
    SERIAL_PAIR("NOTICE: p-top message sent on attempt ", numAttempts);
    SERIAL_PAIR(" of ", maxAttempts);
    SERIAL_EOL;
  }

  return returnValue;
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

PTopPin::Sample PTopPin::value()     { return Sample( adcSamples.value()     ); }
PTopPin::Sample PTopPin::readValue() { return Sample( adcSamples.readValue() ); }
