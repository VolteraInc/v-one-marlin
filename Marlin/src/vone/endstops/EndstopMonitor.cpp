#include "EndstopMonitor.h"

#include "Endstop.h"
#include "Endstops.h"

EndstopMonitor::EndstopMonitor(Endstops& endstops): m_endstops(endstops) {
}

void EndstopMonitor::ignoreToolSwitch(bool ignore) {
  ScopedInterruptDisable sid;
  m_ignoreToolSwitch = ignore;
}

void EndstopMonitor::ignoreZSwitch(bool ignore) {
  ScopedInterruptDisable sid;
  m_ignoreZSwitch = ignore;
}

void EndstopMonitor::ignoreXYPositionerSwitches(bool ignore) {
  ScopedInterruptDisable sid;
  m_ignoreXYPositionerSwitches = ignore;
}

EndstopFilter* EndstopMonitor::lookup(const Endstop& endstop) {
  switch (endstop.pin) {
    case X_MIN_PIN: return &m_xMin;
    case Y_MIN_PIN: return &m_yMin;
    case Z_MAX_PIN: return &m_zMax;

    case Z_MIN_PIN: return &m_zSwitch;

    case XY_MIN_X_PIN: return &m_xyMinX;
    case XY_MAX_X_PIN: return &m_xyMaxX;
    case XY_MIN_Y_PIN: return &m_xyMinY;
    case XY_MAX_Y_PIN: return &m_xyMaxY;

    case P_BOT_PIN: return &m_calibrationPlate;

    case P_TOP_PIN: return &m_toolSwitch;
    default:
      return nullptr;
  }
}

bool EndstopMonitor::isEndstopTriggered(const Endstop& endstop) const {
  const auto* es = lookup(endstop);
  if (!es) {
    logError << "Unable to determine if end stop triggered, unrecognized endstop provided" << endl;
    // Treat read failures as TRIGGERED, so that motion stops
    // Note: not sure if this is the right thing to do, but it's
    // safer than allowing motion to continue. Also, if reads are
    // failing there is a logic bug
    return true;
  }
  return es->triggered();
}

bool EndstopMonitor::isEndstopTriggered(enum AxisEnum axis, int direction) const {
  switch (axis) {
    case X_AXIS: return direction == -1 ? isTriggeredRight() : isTriggeredLeft();
    case Y_AXIS: return direction == -1 ? isTriggeredBack() : isTriggeredForward();
    case Z_AXIS: return direction == -1 ? isTriggeredDown() : isTriggeredUp();
  }
}

// void EndstopMonitor::resetEndstop(const Endstop& endstop) {
//   auto es = lookup(endstop);
//   if (!es) {
//     logError << "Unable to reset end stop, unrecognized endstop provided" << endl;
//     return;
//   }
//   return es->reset();
// }

// void EndstopMonitor::resetEndstop(enum AxisEnum axis, int direction) {
//   switch (axis) {
//     case X_AXIS: direction == -1 ? resetRightEndstops() : resetLeftEndstops();
//     case Y_AXIS: direction == -1 ? resetBackEndstops() : resetForwardEndstops();
//     case Z_AXIS: direction == -1 ? resetDownEndstops() : resetUpEndstops();
//   }
// }

int EndstopMonitor::acknowledgeEndstopTriggered(const Endstop& endstop) {
  // if (!isEndstopTriggered(endstop)) {
  //   return -1;
  // }
  // resetEndstop(endstop);
  // return 0;

  if (
    m_triggerLog.empty() ||
    m_triggerLog.front().endstop.pin != endstop.pin
  ) {
    return -1;
  }
  m_triggerLog.pop();
  return 0;
}

void EndstopMonitor::reportEndstopHits(
  void (*reportHit)(const Endstop& endstop, long stepWhenTriggered)
) {
  if ( 1) {

  } else {
    // multiple it detected
  }

  while(!m_triggerLog.empty()) {
    const auto& tg = m_triggerLog.front();
    reportHit(tg.endstop, tg.step);
  }
}
