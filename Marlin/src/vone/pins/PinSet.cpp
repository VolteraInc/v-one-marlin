#include "PinSet.h"

#include "../../../serial.h"
#include "../../../Marlin.h"


PinSet::PinSet(
  int ptopDigitalPin,
  int ptopAnalogPin,
  int bedTemperatureAnalogPin,
  int heaterDigitalPin
)
  : ptop(ptopDigitalPin, ptopAnalogPin)
  , bedTemperature(bedTemperatureAnalogPin)
  , heater(heaterDigitalPin)
{
  // LED Pins
  //SET_OUTPUT(LED_RED_PIN);
  //WRITE(LED_RED_PIN, LOW);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);


  //SET_OUTPUT(LED_GREEN_PIN);
  //WRITE(LED_GREEN_PIN, LOW);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);

  //SET_OUTPUT(LED_BLUE_PIN);
  //WRITE(LED_BLUE_PIN, LOW);
  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);
}
