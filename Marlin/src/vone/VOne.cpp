#include "VOne.h"

VOne::VOne(
  int ptopDigialPin,
  int ptopAnalogPin,
  int bedTemperaturePin,
  int heaterDigitalPin
)
  : pins(ptopDigialPin, ptopAnalogPin, bedTemperaturePin, heaterDigitalPin)
  , adc(pins.ptop, pins.bedTemperature)
  , heater(pins.heater, pins.bedTemperature)
  , tools(pins.ptop)
  , toolDetector(tools, stepper, pins.ptop)
{
}
