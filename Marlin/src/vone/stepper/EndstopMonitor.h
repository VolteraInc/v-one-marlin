#pragma once

#include "EndstopFilter.h"

class EndstopMonitor {
  public:
    EndstopMonitor();

    inline void ignoreToolSwitch(bool ignore = true);
    inline void ignoreZSwitch(bool ignore = true);
    inline void ignoreXYPositionerSwitches(bool ignore = true);

    bool isEndstopTriggered(const Endstop& endstop);
    void resetEndstop(const Endstop& endstop);

    // X-Axis
    inline void onMovingRight();
    inline void onMovingLeft();

    // Y-Axis
    inline void onMovingBack();
    inline void onMovingForward();

    // Z-Axis
    inline void onMovingUp();
    inline void onMovingDown();

  private:
    volatile bool m_ignoreToolSwitch = true;
    volatile bool m_ignoreZSwitch = true;
    volatile bool m_ignoreXYPositionerSwitches = true;

    EndstopFilter m_xMin;
    EndstopFilter m_yMin;

    EndstopFilter m_zMin;
    EndstopFilter m_zMax;

    EndstopFilter m_zSwitch;

    EndstopFilter m_xyMinX;
    EndstopFilter m_xyMaxX;

    EndstopFilter m_xyMinY;
    EndstopFilter m_xyMaxY;

    EndstopFilter m_calibrationPlate;

    EndstopFilter m_toolSwitch;

    const EndstopFilter& lookup(const Endstop& endstop);
};

void EndstopMonitor::ignoreToolSwitch(bool ignore) {
  CRITICAL_SECTION_START;
  m_ignoreToolSwitch = ignore;
  CRITICAL_SECTION_END;
}

void EndstopMonitor::ignoreZSwitch(bool ignore) {
  CRITICAL_SECTION_START;
  m_ignoreZSwitch = ignore;
  CRITICAL_SECTION_END;
}

void EndstopMonitor::ignoreXYPositionerSwitches(bool ignore) {
  CRITICAL_SECTION_START;
  m_ignoreXYPositionerSwitches = ignore;
  CRITICAL_SECTION_END;
}

const EndstopFilter& EndstopMonitor::lookup(const Endstop& endstop) {
  switch (endstop.pin) {
    case X_MIN_PIN: return m_xMin;
    case Y_MIN_PIN: return m_xMin;

    case Z_MIN_PIN: return m_xMin;
    case Z_MAX_PIN: return m_xMin;

    case XY_MIN_X_PIN: return m_xyMinX;
    case XY_MIN_X_PIN: return m_xyMinX;
    case XY_MIN_X_PIN: return m_xyMinX;
    case XY_MIN_X_PIN: return m_xyMinX;

    case X_MIN_PIN: return m_xMin;
    case X_MIN_PIN: return m_xMin;
    case X_MIN_PIN: return m_toolSwitch;
    default:
      logError <<
  }
}

bool EndstopMonitor::isEndstopTriggered(const Endstop& endstop) {
  const auto& es = lookup(endStop);
  ScopedInterruptDisable sid;
  return es.triggered();
}

void EndstopMonitor::resetEndstop(const Endstop& endstop) {
  const auto& es = lookup(endStop);
  ScopedInterruptDisable sid;
  return es.reset();
}

void EndstopMonitor::onMovingRight() {
  m_xMin.addSample(READ_PIN(X_MIN));

  if (m_ignoreXYPositionerSwitches) {
    m_xyMinX.reset();
  } else {
    m_xyMinX.addSample(READ_PIN(XY_MIN_Y));
  }

  m_xyMaxX.reset();
}

void EndstopMonitor::onMovingLeft() {
  m_xMin.reset();

  m_xyMinX.reset();

  if (m_ignoreXYPositionerSwitches) {
    m_xyMaxX.reset();
  } else {
    m_xyMaxX.addSample(READ_PIN(XY_MAX_X));
  }
}

void EndstopMonitor::onMovingBack() {
  m_yMin.addSample(READ_PIN(Y_MIN));

  if (m_ignoreXYPositionerSwitches) {
    m_xyMinY.reset();
  } else {
    m_xyMinY.addSample(READ_PIN(XY_MIN_Y));
  }

  m_xyMaxY.reset();
}

void EndstopMonitor::onMovingForward() {
  m_yMin.reset()

  m_xyMinY.reset();

  if (m_ignoreXYPositionerSwitches) {
    m_xyMaxY.reset();
  } else {
    m_xyMaxY.addSample(READ_PIN(XY_MAX_Y));
  }
}

void EndstopMonitor::onMovingUp() {
  m_zMax.addSample(READ_PIN(Z_MAX));
  m_zSwitch.reset();
  m_calibrationPlate.reset();
  m_toolSwitch.reset();
}

void EndstopMonitor::onMovingDown() {
  m_zMax.reset();

  if (m_ignoreZSwitch) {
    m_zSwitch.reset();
  } else {
    m_zSwitch.addSample(READ_PIN(Z_MIN));
  }

  if (m_ignoreXYPositionerSwitches) {
    m_calibrationPlate.reset();
  } else {
    m_calibrationPlate.addSample(READ_PIN(P_BOT));
  }

  if (m_ignoreToolSwitch) {
    m_toolSwitch.reset();
  } else {
    bool ptop = false;
    if (m_pins.ptop.readDigitalValue(ptop)) {
      // Treat read failures as TRIGGERED, so that motion stops
      // Note: not sure if this is the right thing to do, but it's
      // safer than allowing motion to continue. Also, if reads are
      // failing there is a bug
      ptop = true;
    }
    m_toolSwitch.addSample(ptop);
  }
}

bool EndstopMonitor::triggered() {
  return (
    m_xMin.triggered() ||
    m_yMin.triggered() ||
    m_zMax.triggered() ||

    m_xyMinX.triggered() ||
    m_xyMaxX.triggered() ||
    m_xyMinY.triggered() ||
    m_xyMaxY.triggered() ||

    m_zSwitch.triggered() ||
    m_calibrationPlate.triggered() ||
    m_toolSwitch.triggered()
  );
}
