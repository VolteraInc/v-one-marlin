#include "EndstopMonitor.h"

#include "Endstop.h"
#include "Endstops.h"

EndstopMonitor::EndstopMonitor(
  Endstops& endstops
) : m_endstops(endstops)
{
  m_toolSwitch.ignore();
  m_zSwitch.ignore();
  m_calibrationPlate.ignore();

  m_xyPositionerRight.ignore();
  m_xyPositionerLeft.ignore();
  m_xyPositionerBack.ignore();
  m_xyPositionerForward.ignore();
}

const EndstopFilter* EndstopMonitor::lookup(const Endstop& endstop) const {
  switch (endstop.pin) {
    case X_MIN_PIN: return &m_xMin;
    case Y_MIN_PIN: return &m_yMin;
    case Z_MAX_PIN: return &m_zMax;

    case Z_MIN_PIN: return &m_zSwitch;

    case XY_MIN_X_PIN: return &m_xyPositionerRight;
    case XY_MAX_X_PIN: return &m_xyPositionerLeft;
    case XY_MIN_Y_PIN: return &m_xyPositionerBack;
    case XY_MAX_Y_PIN: return &m_xyPositionerForward;

    case P_BOT_PIN: return &m_calibrationPlate;

    case P_TOP_PIN: return &m_toolSwitch;
    default:
      logError
        << F("Unable to access endstop with pin ")
        << endstop.pin
        << endl;
      return nullptr;
  }
}

EndstopFilter* EndstopMonitor::lookup(const Endstop& endstop) {
  // Implement a non-const version of lookup using the const version
  // ref https://stackoverflow.com/questions/123758/how-do-i-remove-code-duplication-between-similar-const-and-non-const-member-func
  const auto& cThis = *this;
  return const_cast<EndstopFilter*>(cThis.lookup(endstop));
}

void EndstopMonitor::ignore(const Endstop& endstop, bool ignore) {
  auto filter = lookup(endstop);
  if (!filter) {
    return;
  }
  filter->ignore(ignore);

  if (logging_enabled) {
    log
      << endstop.name
      << (ignore ? F(" disabled") : F(" enabled"))
      << endl;
  }
}

bool EndstopMonitor::ignoring(const Endstop& endstop) const {
  auto filter = lookup(endstop);
  if (!filter) {
    return true;
  }
  return filter->ignored();
}

bool EndstopMonitor::isTriggered(const Endstop& endstop) const {
  const auto* es = lookup(endstop);
  if (!es) {
    logError
      << F("Unable to determine if end stop triggered, unrecognized endstop provided")
      << endl;
    // Treat read failures as TRIGGERED, so that motion stops
    // Note: not sure if this is the right thing to do, but it's
    // safer than allowing motion to continue. Also, if reads are
    // failing there is a logic bug
    return true;
  }

  return es->triggered();
}

bool EndstopMonitor::isTriggered(enum AxisEnum axis) const {
  switch (axis) {
    case X_AXIS: return isTriggeredRight() || isTriggeredLeft();
    case Y_AXIS: return isTriggeredBack() || isTriggeredForward();
    case Z_AXIS: return isTriggeredDown() || isTriggeredUp();
    case E_AXIS: return false;
  }
  return false;
}

bool EndstopMonitor::isTriggered(enum AxisEnum axis, int direction) const {
  switch (axis) {
    case X_AXIS: return direction == -1 ? isTriggeredRight() : isTriggeredLeft();
    case Y_AXIS: return direction == -1 ? isTriggeredBack() : isTriggeredForward();
    case Z_AXIS: return direction == -1 ? isTriggeredDown() : isTriggeredUp();
    case E_AXIS: return false;
  }
  return false;
}

int EndstopMonitor::acknowledgeTriggered(const Endstop& endstop) {
  if (
    m_triggerLog.size() != 1 ||
    m_triggerLog.front()->endstop->pin != endstop.pin
  ) {
    return -1;
  }
  m_triggerLog.pop();
  return 0;
}

int EndstopMonitor::acknowledgeTriggered(enum AxisEnum axis, int direction) {
  if (
    m_triggerLog.size() != 1 ||
    m_triggerLog.front()->endstop->axis != axis ||
    m_triggerLog.front()->endstop->direction != direction
  ) {
    return -1;
  }
  m_triggerLog.pop();
  return 0;
}

void EndstopMonitor::reportHits(
  void (*reportHits)(const Endstop& endstop, float triggeringPosition)
) {
  while(!m_triggerLog.empty()) {
    const auto& tg = *m_triggerLog.front();
    reportHits(*tg.endstop, tg.positionInAxis);
    m_triggerLog.pop();
  }
}
