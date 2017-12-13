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

  auto ready = adcSamples.add(value);
  if (ready) {
    // Note: Why 10ms?
    // Making this value too low leaves less time for other tasks
    // Making it too high an we may not have enough resolution to 
    // detect signals.
    // 10ms seems comfortable given that the router's
    // acknowledgement is 100ms and its reset signal is 500ms.
    preventAdcSamplingUntil = millis() + 10;

    setMode_Idle();
  }
}
