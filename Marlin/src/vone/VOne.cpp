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

  , m_endstopMonitor(endstops)
  , stepper(m_endstopMonitor)

  , toolBox(stepper, pins.ptop, endstops.toolSwitch)
  , toolDetector(toolBox, pins.ptop)
{
}
