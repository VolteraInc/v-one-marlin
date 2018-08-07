#include "PinSet.h"

#include "../../../serial.h"
#include "../../../Marlin.h"


PinSet::PinSet(
  int ptopDigialPin,
  int ptopAnalogPin,
  int bedTemperatureAnalogPin,
  int heaterDigitalPin
)
  : ptop(ptopDigialPin, ptopAnalogPin)
  , bedTemperature(bedTemperatureAnalogPin)
  , heater(heaterDigitalPin)
{
}
