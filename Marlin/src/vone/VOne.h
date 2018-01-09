#pragma once

#include "pins/PinSet.h"
#include "pins/adc/AnalogDigitalConverter.h"
#include "bed/heater/Heater.h"
#include "stepper/Stepper.h"

#include "tools/ToolBox.h"

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
    tools::ToolBox toolBox;

    // Perform work that must happen frequently but can be
    // interrupted (briefly) by time critical work like
    // serial character reads and the stepper.
    // Note: work done in the main loop can be delayed delayed by
    //       10s, 30s or longer. It depends on how long it takes
    //       to process the current command.
    inline void frequentInterruptibleWork() {
      // Allow other interrupts
      CBI(TIMSK0, OCIE0B);
      sei();

      // Updated ADC values are needed by heater and toolDetector
      adc.frequentInterruptibleWork();

      // Heater management
      // Note: Delaying heater updates (even for a few seconds) could
      //       result in several degrees of overshoot
      // TODO: temperature profiles should be processed here
      //       otherwise the next step in the profile can be delayed
      //       by command processing
      heater.frequentInterruptibleWork();

      SBI(TIMSK0, OCIE0B);
    }

};
