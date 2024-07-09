#include "Endstops.h"

#include "../../../serial.h"

#include <WString.h>
#include "../pins/PTopPin/PTopPin.h"


Endstops::Endstops(ZSwitch::Type zSwitchType)
  : xMin(F("right (x-min)"), X_MIN_PIN, X_AXIS, -1, X_MIN_ENDSTOP_INVERTING, false)
  , yMin(F("back (y-min)"), Y_MIN_PIN, Y_AXIS, -1, Y_MIN_ENDSTOP_INVERTING, false)
  , zMax(F("top (z-max)"), Z_MAX_PIN, Z_AXIS, 1, Z_MAX_ENDSTOP_INVERTING, false)
  , calibrationPlate(F("calibration plate (p-bot)"), P_BOT_PIN, Z_AXIS, -1, P_BOT_ENDSTOP_INVERTING, false)
  , toolSwitch(F("tool switch (p-top)"), P_TOP_PIN, Z_AXIS, -1, P_TOP_ENDSTOP_INVERTING, false)

  , zSwitch(zSwitchType, F("z-switch (z-min)"), Z_MIN_PIN, Z_AXIS, -1, Z_MIN_ENDSTOP_INVERTING, Z_MIN_VIRTUAL_ENDSTOP)
  , xyPositionerLeft(F("xy-positioner left (xy-max-x)"), XY_MAX_X_PIN, X_AXIS, 1, XY_MAX_X_ENDSTOP_INVERTING, XY_MAX_X_VIRTUAL_ENDSTOP)
  , xyPositionerRight(F("xy-positioner right (xy-min-x)"), XY_MIN_X_PIN, X_AXIS, -1, XY_MIN_X_ENDSTOP_INVERTING, XY_MIN_X_VIRTUAL_ENDSTOP)
  , xyPositionerBack(F("xy-positioner back (xy-min-y)"), XY_MIN_Y_PIN, Y_AXIS, -1, XY_MIN_Y_ENDSTOP_INVERTING, XY_MIN_Y_VIRTUAL_ENDSTOP)
  , xyPositionerForward(F("xy-positioner front (xy-max-y)"), XY_MAX_Y_PIN, Y_AXIS, 1, XY_MAX_Y_ENDSTOP_INVERTING, XY_MAX_Y_VIRTUAL_ENDSTOP)
   
  #ifdef TRINAMIC_MOTORS
    //previously called xLim and yLim, these are the software endstops to detect the maximum left and front extent of machine
  , xMax(F("left (x-max)"), X_LIM_PIN, X_AXIS, 1, X_LIM_ENDSTOP_INVERTING, false)
  , yMax(F("front (y-max)"), Y_LIM_PIN, Y_AXIS, 1, Y_LIM_ENDSTOP_INVERTING, false)
   #endif
{
  pinMode(P_BOT_PIN, INPUT_PULLUP); //To stop P-Bot from floating, previously an external resistor
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

    #ifdef TRINAMIC_MOTORS
    case X_LIM_PIN: return &xMax;
    case Y_LIM_PIN: return &yMax;
    #endif

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

  log << sp << s_pinStatusToString(READ_PIN(P_BOT)) << sp << calibrationPlate.pin << sp << calibrationPlate.name << endl;
  log << sp << s_pinStatusToString(READ_PIN(P_TOP)) << sp << toolSwitch.pin       << sp << toolSwitch.name       << endl;

  #ifndef XYZ_STRAIN //remove this since we are no longer reading the pin for trigger
  log << sp << s_pinStatusToString(READ_PIN(Z_MIN)) << sp << zSwitch.pin          << sp << zSwitch.name          << endl;
  log << sp << s_pinStatusToString(READ_PIN(XY_MIN_X)) << sp << xyPositionerRight.pin   << sp << xyPositionerRight.name   << endl;
  log << sp << s_pinStatusToString(READ_PIN(XY_MAX_X)) << sp << xyPositionerLeft.pin    << sp << xyPositionerLeft.name    << endl;
  log << sp << s_pinStatusToString(READ_PIN(XY_MIN_Y)) << sp << xyPositionerBack.pin    << sp << xyPositionerBack.name    << endl;
  log << sp << s_pinStatusToString(READ_PIN(XY_MAX_Y)) << sp << xyPositionerForward.pin << sp << xyPositionerForward.name << endl;
  #endif

  #ifdef TRINAMIC_MOTORS
  log << sp << s_pinStatusToString(READ_PIN(X_LIM)) << sp << xMax.pin << sp << xMax.name << endl;
  log << sp << s_pinStatusToString(READ_PIN(Y_LIM)) << sp << yMax.pin << sp << yMax.name << endl;
  #endif
}

static void s_reportAndUpdateStatus(
  const __FlashStringHelper* name,
  bool& reportedStatus,
  bool currentStatus
) {
  if (reportedStatus != currentStatus) {
    reportedStatus = currentStatus;
    log << name << s_pinStatusToString(currentStatus) << endl;
  }
}

void Endstops::reportChanges() {
  s_reportAndUpdateStatus(xMin.name, m_reportedStatus.xMinTriggered, READ_PIN(X_MIN));
  s_reportAndUpdateStatus(yMin.name, m_reportedStatus.yMinTriggered, READ_PIN(Y_MIN));
  s_reportAndUpdateStatus(zMax.name, m_reportedStatus.zMaxTriggered, READ_PIN(Z_MAX));

  s_reportAndUpdateStatus(calibrationPlate.name, m_reportedStatus.calibrationPlateTriggered, READ_PIN(P_BOT));
  s_reportAndUpdateStatus(toolSwitch.name, m_reportedStatus.toolSwitchTriggered, READ_PIN(P_TOP));

  #ifndef XYZ_STRAIN //remove this since we are no longer reading the pin for trigger
  s_reportAndUpdateStatus(zSwitch.name, m_reportedStatus.zSwitchTriggered, READ_PIN(Z_MIN));
  s_reportAndUpdateStatus(xyPositionerLeft.name, m_reportedStatus.xyPositionerLeftTriggered, READ_PIN(XY_MAX_X));
  s_reportAndUpdateStatus(xyPositionerRight.name, m_reportedStatus.xyPositionerRightTriggered, READ_PIN(XY_MIN_X));
  s_reportAndUpdateStatus(xyPositionerBack.name, m_reportedStatus.xyPositionerBackTriggered, READ_PIN(XY_MIN_Y));
  s_reportAndUpdateStatus(xyPositionerForward.name, m_reportedStatus.xyPositionerForwardTriggered, READ_PIN(XY_MAX_Y));
  #endif
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
  
  protocol << F("z_max: ") << s_deprecatedFormat(READ_PIN(Z_MAX)) << endl;

  protocol << F("p_top: ") << s_deprecatedFormat(READ_PIN(P_TOP)) << endl;
  protocol << F("p_bot: ") << s_deprecatedFormat(READ_PIN(P_BOT)) << endl;

  #ifndef XYZ_STRAIN //remove this since we are no longer reading the pin for trigger
  protocol << F("z_min: ") << s_deprecatedFormat(READ_PIN(Z_MIN)) << endl;
  protocol << F("xy_min_x: ") << s_deprecatedFormat(READ_PIN(XY_MIN_X)) << endl;
  protocol << F("xy_max_x: ") << s_deprecatedFormat(READ_PIN(XY_MAX_X)) << endl;
  protocol << F("xy_min_y: ") << s_deprecatedFormat(READ_PIN(XY_MIN_Y)) << endl;
  protocol << F("xy_max_y: ") << s_deprecatedFormat(READ_PIN(XY_MAX_Y)) << endl;
  #endif
}
