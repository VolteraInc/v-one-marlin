#pragma once

#include "pins/PinSet.h"
#include "pins/adc/AnalogDigitalConverter.h"
#include "bed/heater/Heater.h"
#include "stepper/Stepper.h"


class VOne {
  public:
    VOne(
      int ptopDigialPin,
      int ptopAnalogPin,
      int bedTemperatureAnalogPin,
      int heaterDigitalPin
    );

    PinSet pins;
    adc::AnalogDigitalConverter adc;
    Heater heater;
    Stepper stepper;

    // Perform work that must happen frequently but can be
    // interrupted (briefly) by time critical work like
    // serial character reads and stepper work
    inline void frequentInterruptibleWork() {
      // Allow other interrupts
      CBI(TIMSK0, OCIE0B);
      sei();

      adc.frequentInterruptibleWork();
      heater.frequentInterruptibleWork();

      SBI(TIMSK0, OCIE0B);
    }
};
