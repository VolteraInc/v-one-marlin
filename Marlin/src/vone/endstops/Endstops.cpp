#include "Endstops.h"

#include "../../../serial.h"
#include "../../../Marlin.h"

#include <WString.h>
#include "../pins/PTopPin/PTopPin.h"


static const auto isAxisLimit = true;
Endstops::Endstops()
  : xMin(F("right (x-min)"), X_MIN_PIN, X_AXIS, -1, X_MIN_ENDSTOP_INVERTING, isAxisLimit)
  , yMin(F("back (y-min)"), Y_MIN_PIN, Y_AXIS, -1, Y_MIN_ENDSTOP_INVERTING, isAxisLimit)
  , zMax(F("top (z-max)"), Z_MAX_PIN, Z_AXIS, 1, Z_MAX_ENDSTOP_INVERTING, isAxisLimit)
  , zSwitch(F("z-switch (z-min)"), Z_MIN_PIN, Z_AXIS, -1, Z_MIN_ENDSTOP_INVERTING)
  , xyPositionerLeft(F("xy-positioner left (xy-max-x)"), XY_MAX_X_PIN, X_AXIS, 1, XY_MAX_X_ENDSTOP_INVERTING)
  , xyPositionerRight(F("xy-positioner right (xy-min-x)"), XY_MIN_X_PIN, X_AXIS, -1, XY_MIN_X_ENDSTOP_INVERTING)
  , xyPositionerBack(F("xy-positioner back (xy-min-y)"), XY_MIN_Y_PIN, Y_AXIS, -1, XY_MIN_Y_ENDSTOP_INVERTING)
  , xyPositionerForward(F("xy-positioner front (xy-max-y)"), XY_MAX_Y_PIN, Y_AXIS, 1, XY_MAX_Y_ENDSTOP_INVERTING)
  , calibrationPlate(F("calibration plate (p-bot)"), P_BOT_PIN, Z_AXIS, -1, P_BOT_ENDSTOP_INVERTING)
  , toolSwitch(F("tool switch (p-top)"), P_TOP_PIN, Z_AXIS, -1, P_TOP_ENDSTOP_INVERTING)
  #ifdef TRINAMIC_DRIVERS
  , xLim(F("x-motor (x-lim)"), X_LIM_PIN, X_AXIS, -1, X_LIM_ENDSTOP_INVERTING, isAxisLimit)
  , yLim(F("y-motor (y-lim)"), Y_LIM_PIN, Y_AXIS, -1, Y_LIM_ENDSTOP_INVERTING, isAxisLimit)
  #endif // TRINAMIC_DRIVERS
{
}

const Endstop* Endstops::lookup(const int pin) const {
  switch (pin) {
    case X_MIN_PIN: return &xMin;
    case Y_MIN_PIN: return &yMin;
    case Z_MAX_PIN: return &zMax;

    case Z_MIN_PIN: return &zSwitch;

    case XY_MIN_X_PIN: return &xyPositionerRight;
    case XY_MAX_X_PIN: return &xyPositionerLeft;
    case XY_MIN_Y_PIN: return &xyPositionerBack;
    case XY_MAX_Y_PIN: return &xyPositionerForward;

    case P_BOT_PIN: return &calibrationPlate;

    case P_TOP_PIN: return &toolSwitch;

    #ifdef TRINAMIC_DRIVERS
    case X_LIM_PIN: return &xLim;
    case Y_LIM_PIN: return &yLim;
    #endif // TRINAMIC_DRIVERS

    default:
      return nullptr;
  }
}

static const __FlashStringHelper* s_pinStatusToString(bool isTriggered) {
  return isTriggered ? F(" TRIGGERED") : F(" open");
}

void Endstops::outputStatus() const {
  const auto sp = F("  ");
  log << F("Endstop status") << endl;
  log << sp << F("status pin name") << endl;
  log << sp << s_pinStatusToString(READ_PIN(X_MIN)) << sp << xMin.pin << sp << xMin.name << endl;
  log << sp << s_pinStatusToString(READ_PIN(Y_MIN)) << sp << yMin.pin << sp << yMin.name  << endl;
  log << sp << s_pinStatusToString(READ_PIN(Z_MAX)) << sp << zMax.pin << sp << zMax.name << endl;

  log << sp << s_pinStatusToString(READ_PIN(Z_MIN)) << sp << zSwitch.pin          << sp << zSwitch.name          << endl;
  log << sp << s_pinStatusToString(READ_PIN(P_BOT)) << sp << calibrationPlate.pin << sp << calibrationPlate.name << endl;
  log << sp << s_pinStatusToString(READ_PIN(P_TOP)) << sp << toolSwitch.pin       << sp << toolSwitch.name       << endl;

  log << sp << s_pinStatusToString(READ_PIN(XY_MIN_X)) << sp << xyPositionerRight.pin   << sp << xyPositionerRight.name   << endl;
  log << sp << s_pinStatusToString(READ_PIN(XY_MAX_X)) << sp << xyPositionerLeft.pin    << sp << xyPositionerLeft.name    << endl;
  log << sp << s_pinStatusToString(READ_PIN(XY_MIN_Y)) << sp << xyPositionerBack.pin    << sp << xyPositionerBack.name    << endl;
  log << sp << s_pinStatusToString(READ_PIN(XY_MAX_Y)) << sp << xyPositionerForward.pin << sp << xyPositionerForward.name << endl;

  #ifdef TRINAMIC_DRIVERS
  log << sp << s_pinStatusToString(READ_PIN(X_LIM)) << sp << xLim.pin << sp << xLim.name << endl;
  log << sp << s_pinStatusToString(READ_PIN(Y_LIM)) << sp << yLim.pin << sp << yLim.name  << endl;
  #endif // TRINAMIC_DRIVERS
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

  #ifdef TRINAMIC_DRIVERS
  protocol << F("x_lim: ") << s_deprecatedFormat(READ_PIN(X_LIM)) << endl;
  protocol << F("y_lim: ") << s_deprecatedFormat(READ_PIN(Y_LIM)) << endl;
  #endif // TRINAMIC_DRIVERS
}
