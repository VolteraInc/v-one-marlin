#include "VOne.h"

VOne::VOne(
  int ptopDigitalPin,
  int ptopAnalogPin,
  int bedTemperaturePin,
  int heaterDigitalPin
)
  : pins(ptopDigitalPin, ptopAnalogPin, bedTemperaturePin, heaterDigitalPin)
  , adc(pins.ptop, pins.bedTemperature)
  , heater(pins.heater, pins.bedTemperature)
  , toolBox(stepper, pins.ptop)
  , toolDetector(toolBox, pins.ptop)
{
}
