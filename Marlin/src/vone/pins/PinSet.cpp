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
  // LED Pins
  SET_OUTPUT(LED_RED_PIN);
  WRITE(LED_RED_PIN, LOW);

  SET_OUTPUT(LED_GREEN_PIN);
  WRITE(LED_GREEN_PIN, LOW);

  SET_OUTPUT(LED_BLUE_PIN);
  WRITE(LED_BLUE_PIN, LOW);

  // Initialize Stepper motor pins
  // Note: each motor has 3 pins:
  //  - enable/disable
  //  - step
  //  - step direction
  SET_OUTPUT(X_ENABLE_PIN);
  disable_x();
  SET_OUTPUT(X_STEP_PIN);
  WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
  SET_OUTPUT(X_DIR_PIN);

  SET_OUTPUT(Y_ENABLE_PIN);
  disable_y();
  SET_OUTPUT(Y_STEP_PIN);
  WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
  SET_OUTPUT(Y_DIR_PIN);

  SET_OUTPUT(Z_ENABLE_PIN);
  disable_z();
  SET_OUTPUT(Z_STEP_PIN);
  WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
  SET_OUTPUT(Z_DIR_PIN);

  SET_OUTPUT(E_ENABLE_PIN);
  disable_e();
  SET_OUTPUT(E_STEP_PIN);
  WRITE(E_STEP_PIN, INVERT_E_STEP_PIN);
  SET_OUTPUT(E_DIR_PIN);
}
