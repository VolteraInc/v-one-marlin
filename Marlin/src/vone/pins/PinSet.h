#pragma once

#include "PTopPin/PTopPin.h"
#include "BedTemperaturePin/BedTemperaturePin.h"
#include "HeaterPin.h"

struct PinSet {
  PTopPin ptop;
  BedTemperaturePin bedTemperature;
  HeaterPin heater;

  PinSet(
    int ptopDigitalPin,
    int ptopAnalogPin,
    int bedTemperatureAnalogPin,
    int heaterDigitalPin
  );
};
