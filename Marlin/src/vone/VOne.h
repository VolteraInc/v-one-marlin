#pragma once

#include "pins/PTopPin/PTopPin.h"
#include "pins/BedTemperaturePin/BedTemperaturePin.h"
#include "pins/HeaterPin.h"

#include "pins/adc/AnalogDigitalConverter.h"
#include "bed/heater/Heater.h"

class VOne {
  public:
    VOne(
      int ptopDigialPin,
      int ptopAnalogPin,
      int bedTemperatureAnalogPin,
      int heaterDigitalPin
    );

    struct Pins {
      PTopPin ptop;
      BedTemperaturePin bedTemperature;
      HeaterPin heater;

      Pins(
        int ptopDigialPin,
        int ptopAnalogPin,
        int bedTemperatureAnalogPin,
        int heaterDigitalPin
      );


      int outputEndStopStatus();
    } pins;

    adc::AnalogDigitalConverter adc;
    Heater heater;
    // Tool& currentTool;

    // Perform work that needs to happen periodicially but
    // can be interrupted by serial and stepper work
    inline void periodicInterruptibleWork() {
      // Allow other interrupts
      CBI(TIMSK0, OCIE0B);
      sei();

      adc.periodicInterruptibleWork();

      heater.periodicInterruptibleWork();

      SBI(TIMSK0, OCIE0B);
    }
};
