#include "PinSet.h"

#include "../../../serial.h"
#include "../../../Marlin.h"

#include "../stepper/trinamic.h"

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
  // Initialize Trinamic Driver.
  #ifdef TRINAMIC_DRIVERS
  trinamicInit(); // trinamicInit initalizes all the pins for us.
  #endif

  // Endstops and pullups
  SET_INPUT(X_MIN_PIN);
  SET_INPUT(Y_MIN_PIN);
  SET_INPUT(Z_MIN_PIN);
  SET_INPUT(Z_MAX_PIN);

  // Endstops and pullups
  #ifdef TRINAMIC_SENSORLESS
  SET_INPUT(X_LIM_PIN);
  SET_INPUT(Y_LIM_PIN);
  SET_INPUT(E_LIM_PIN);
  #endif
  SET_INPUT(P_BOT_PIN);

  SET_INPUT(XY_MIN_X_PIN);
  SET_INPUT(XY_MAX_X_PIN);
  SET_INPUT(XY_MIN_Y_PIN);
  SET_INPUT(XY_MAX_Y_PIN);

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

static const __FlashStringHelper* s_pinStatusToString(bool isTriggered) {
  return isTriggered ? F("TRIGGERED") : F("open");
}

int PinSet::outputEndStopStatus() {
  bool ptopValue = false;
  int returnValue = ptop.readDigitalValue(ptopValue);

  log << F("Endstop status") << endl;
  log << F("  right (x-min): ") << s_pinStatusToString(READ_PIN(X_MIN)) << endl;
  log << F("  back (y-min): ") << s_pinStatusToString(READ_PIN(Y_MIN)) << endl;

  log << F("  z-switch (z-min): ") << s_pinStatusToString(READ_PIN(Z_MIN)) << endl;
  log << F("  top (z-max): ") << s_pinStatusToString(READ_PIN(Z_MAX)) << endl;
  log << F("  calibration plate (p-bot): ") << s_pinStatusToString(READ_PIN(P_BOT)) << endl;
  log << F("  tool switch (p-top): ") << s_pinStatusToString(ptopValue) << endl;

  log << F("  xy-positioner right (xy-min-x): ") << s_pinStatusToString(READ_PIN(XY_MIN_X)) << endl;
  log << F("  xy-positioner left (xy-max-x): ") << s_pinStatusToString(READ_PIN(XY_MAX_X)) << endl;
  log << F("  xy-positioner back (xy-min-y): ") << s_pinStatusToString(READ_PIN(XY_MIN_Y)) << endl;
  log << F("  xy-positioner front (xy-max-y): ") << s_pinStatusToString(READ_PIN(XY_MAX_Y)) << endl;

  #ifdef TRINAMIC_SENSORLESS
  log << F("  x-motor (x-lim): ") << s_pinStatusToString(READ_PIN(X_LIM)) << endl;
  log << F("  y-motor (y-lim): ") << s_pinStatusToString(READ_PIN(Y_LIM)) << endl;
  log << F("  e-motor (e-lim): ") << s_pinStatusToString(READ_PIN(E_LIM)) << endl;
  #endif
  return returnValue;
}

// -----------------------------------------------------------------------
// Note: this output is used in manufacturing scripts
//       so we can't change it without updating those scripts

static const __FlashStringHelper* s_deprecatedFormat(bool isTriggered) {
  return isTriggered ? F("TRIGGERED") : F("open");
}
int PinSet::deprecated_OutputEndStopStatus() {
  bool ptopValue = false;
  int returnValue = ptop.readDigitalValue(ptopValue);

  // Note: this output is used in manufacturing scripts
  protocol << F("Reporting endstop status") << endl;

  protocol << F("x_min: ") << s_deprecatedFormat(READ_PIN(X_MIN)) << endl;
  protocol << F("y_min: ") << s_deprecatedFormat(READ_PIN(Y_MIN)) << endl;

  protocol << F("z_min: ") << s_deprecatedFormat(READ_PIN(Z_MIN)) << endl;
  protocol << F("z_max: ") << s_deprecatedFormat(READ_PIN(Z_MAX)) << endl;

  protocol << F("p_top: ") << s_deprecatedFormat(ptopValue) << endl;
  protocol << F("p_bot: ") << s_deprecatedFormat(READ_PIN(P_BOT)) << endl;

  protocol << F("xy_min_x: ") << s_deprecatedFormat(READ_PIN(XY_MIN_X)) << endl;
  protocol << F("xy_max_x: ") << s_deprecatedFormat(READ_PIN(XY_MAX_X)) << endl;
  protocol << F("xy_min_y: ") << s_deprecatedFormat(READ_PIN(XY_MIN_Y)) << endl;
  protocol << F("xy_max_y: ") << s_deprecatedFormat(READ_PIN(XY_MAX_Y)) << endl;

  return returnValue;
}
