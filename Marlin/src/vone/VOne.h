#pragma once

#include "pins/PTopPin/PTopPin.h"
#include "pins/BedTemperaturePin/BedTemperaturePin.h"
#include "pins/adc/AnalogDigitalConverter.h"

class VOne {
  public:
    VOne(
      int ptopDigialPin,
      int ptopAnalogPin,
      int bedTemperatureAnalogPin
    );

    struct Pins {
      PTopPin ptop;
      BedTemperaturePin bedTemperature;

      adc::AnalogDigitalConverter adc;

      Pins(
        int ptopDigialPin,
        int ptopAnalogPin,
        int bedTemperatureAnalogPin
      );

      int outputEndStopStatus();
    } pins;
};
