#pragma once

#include "pins/PTopPin/PTopPin.h"
#include "pins/BedTemperaturePin/BedTemperaturePin.h"
#include "pins/HeaterPin.h"

#include "pins/adc/AnalogDigitalConverter.h"
#include "bed/heater/Heater.h"

// #include "tools/tools.h"

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
    //ToolCarriage toolCarriage;

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

// To test
//   - analog probing
//   - router comms
//   - temp profiles
// TODO
//   - tool detection
//      - convert toolChanges to OO
//            -- decide on tool-based vs. ToolDetector
//      - confirm no need for super fast change detection (i.e. ~250ms is enough)
