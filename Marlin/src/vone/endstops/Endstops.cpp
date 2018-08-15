#include "Endstops.h"

#include "../../../serial.h"
#include "../../../Marlin.h"

#include <WString.h>
#include "../pins/PTopPin/PTopPin.h"


static const auto isAxisLimit = true;
Endstops::Endstops()
  : xMin(F("right (x-min) endstop"), X_MIN_PIN, X_AXIS, -1, X_MIN_ENDSTOP_INVERTING, isAxisLimit)
  , yMin(F("back (y-min)"), Y_MIN_PIN, Y_AXIS, -1, Y_MIN_ENDSTOP_INVERTING, isAxisLimit)
  , zMax(F("top (z-max)"), Z_MAX_PIN, Z_AXIS, 1, Z_MAX_ENDSTOP_INVERTING, isAxisLimit)
  , zSwitch(F("z-switch (z-min)"), Z_MIN_PIN, Z_AXIS, -1, Z_MIN_ENDSTOP_INVERTING)
  , xyPositionerLeft(F("xy-positioner left (xy-max-x)"), XY_MAX_X_PIN, X_AXIS, 1, XY_MAX_X_ENDSTOP_INVERTING)
  , xyPositionerRight(F("xy-positioner right (xy-min-x)"), XY_MIN_X_PIN, X_AXIS, -1, XY_MIN_X_ENDSTOP_INVERTING)
  , xyPositionerBack(F("xy-positioner back (xy-min-y)"), XY_MIN_Y_PIN, Y_AXIS, -1, XY_MIN_Y_ENDSTOP_INVERTING)
  , xyPositionerForward(F("xy-positioner front (xy-max-y)"), XY_MAX_Y_PIN, Y_AXIS, 1, XY_MAX_Y_ENDSTOP_INVERTING)
  , calibrationPlate(F("calibration plate (p-bot)"), P_BOT_PIN, Z_AXIS, -1, P_BOT_ENDSTOP_INVERTING)
  , toolSwitch(F("tool switch (p-top)"), P_TOP_PIN, Z_AXIS, -1, P_TOP_ENDSTOP_INVERTING)
{
}

static const __FlashStringHelper* s_pinStatusToString(bool isTriggered) {
  return isTriggered ? F("TRIGGERED") : F("open");
}
void Endstops::outputStatus() const {
  const auto indent = F("  ");
  log << F("Endstop status") << endl;
  log << indent << xMin.name << s_pinStatusToString(READ_PIN(X_MIN)) << endl;
  log << indent << yMin.name << s_pinStatusToString(READ_PIN(Y_MIN)) << endl;

  log << indent << zSwitch.name << s_pinStatusToString(READ_PIN(Z_MIN)) << endl;
  log << indent << zMax.name << s_pinStatusToString(READ_PIN(Z_MAX)) << endl;
  log << indent << calibrationPlate.name << s_pinStatusToString(READ_PIN(P_BOT)) << endl;
  log << indent << toolSwitch.name << s_pinStatusToString(READ_PIN(P_TOP)) << endl;

  log << indent << xyPositionerRight.name << s_pinStatusToString(READ_PIN(XY_MIN_X)) << endl;
  log << indent << xyPositionerLeft.name << s_pinStatusToString(READ_PIN(XY_MAX_X)) << endl;
  log << indent << xyPositionerBack.name << s_pinStatusToString(READ_PIN(XY_MIN_Y)) << endl;
  log << indent << xyPositionerForward.name << s_pinStatusToString(READ_PIN(XY_MAX_Y)) << endl;
}

// -----------------------------------------------------------------------
// Note: this output is used in manufacturing scripts
//       so we can't change it without updating those scripts

static const __FlashStringHelper* s_deprecatedFormat(bool isTriggered) {
  return isTriggered ? F("TRIGGERED") : F("open");
}
void Endstops::deprecated_outputStatus() const {
  // Note: this output is used in manufacturing scripts
  protocol << F("Reporting endstop status") << endl;

  protocol << F("x_min: ") << s_deprecatedFormat(READ_PIN(X_MIN)) << endl;
  protocol << F("y_min: ") << s_deprecatedFormat(READ_PIN(Y_MIN)) << endl;

  protocol << F("z_min: ") << s_deprecatedFormat(READ_PIN(Z_MIN)) << endl;
  protocol << F("z_max: ") << s_deprecatedFormat(READ_PIN(Z_MAX)) << endl;

  protocol << F("p_top: ") << s_deprecatedFormat(READ_PIN(P_TOP)) << endl;
  protocol << F("p_bot: ") << s_deprecatedFormat(READ_PIN(P_BOT)) << endl;

  protocol << F("xy_min_x: ") << s_deprecatedFormat(READ_PIN(XY_MIN_X)) << endl;
  protocol << F("xy_max_x: ") << s_deprecatedFormat(READ_PIN(XY_MAX_X)) << endl;
  protocol << F("xy_min_y: ") << s_deprecatedFormat(READ_PIN(XY_MIN_Y)) << endl;
  protocol << F("xy_max_y: ") << s_deprecatedFormat(READ_PIN(XY_MAX_Y)) << endl;
}
