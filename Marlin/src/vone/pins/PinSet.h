#pragma once

#include "PTopPin/PTopPin.h"
#include "BedTemperaturePin/BedTemperaturePin.h"
#include "HeaterPin.h"

struct PinSet {
  PTopPin ptop;
  BedTemperaturePin bedTemperature;
  HeaterPin heater;

  PinSet(
    int ptopDigialPin,
    int ptopAnalogPin,
    int bedTemperatureAnalogPin,
    int heaterDigitalPin
  );

  int outputEndStopStatus();
  int deprecated_OutputEndStopStatus();
};
