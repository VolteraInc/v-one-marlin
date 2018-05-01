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
  log << F("  xy-positioner left (xy_max_x): ") << s_pinStatusToString(READ_PIN(XY_MAX_X)) << endl;
  log << F("  xy-positioner back (xy_min_y): ") << s_pinStatusToString(READ_PIN(XY_MIN_Y)) << endl;
  log << F("  xy-positioner front (xy_max_y): ") << s_pinStatusToString(READ_PIN(XY_MAX_Y)) << endl;

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

  protocol << F("p_top: ") << ptopValue << endl;
  protocol << F("p_bot: ") << s_deprecatedFormat(READ_PIN(P_BOT)) << endl;

  protocol << F("xy_min_x: ") << s_deprecatedFormat(READ_PIN(XY_MIN_X)) << endl;
  protocol << F("xy_max_x: ") << s_deprecatedFormat(READ_PIN(XY_MAX_X)) << endl;
  protocol << F("xy_min_y: ") << s_deprecatedFormat(READ_PIN(XY_MIN_Y)) << endl;
  protocol << F("xy_max_y: ") << s_deprecatedFormat(READ_PIN(XY_MAX_Y)) << endl;

  return returnValue;
}

