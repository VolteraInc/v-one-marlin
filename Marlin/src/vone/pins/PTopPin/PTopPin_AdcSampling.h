#pragma once

#include "PTopPin_main.h"

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
    millis() < delayAdcSamplingUntil ||
    preventAdcSampling
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

  auto ready = adcSamples.add(value);
  if (ready) {
    // adcSamplingDelayMillis impacts how long it takes to read analog values
    // so smaller is better. Too small and it will be constantly toggling the
    // pin mode from input to output, which 'feels' like a bad ideas
    // (toggling every 1ms feels less bad)
    const unsigned int adcSamplingDelay = 1;
    delayAdcSamplingUntil = millis() + adcSamplingDelay;
    setMode_Idle();
  }
}
