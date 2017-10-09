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

  // TODO: still need to detect detaches. Perhaps separate from
  // voltage sampling or maybe not.

  auto ready = adcSamples.add(value);
  if (ready) {
    // Fast enough to detect transitions in Acknowledment signal from drill
    preventAdcSamplingUntil = millis() + 10;

    setMode_Idle();
  }
}
