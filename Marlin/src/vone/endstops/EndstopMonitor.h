#pragma once

#include "../../../Axis.h"
#include "../../utils/ScopedInterruptDisable.h"
#include "../../utils/ScopedInterruptDisable.h"
#include "../pins/PTopPin/PTopPin.h"
#include "EndstopFilter.h"

class Endstop;
class Endstops;

class EndstopMonitor {
  public:
    EndstopMonitor(Endstops& endstops);

    void ignoreToolSwitch(bool ignore = true);
    void ignoreZSwitch(bool ignore = true);
    void ignoreXYPositionerSwitches(bool ignore = true);

    bool isEndstopTriggered(const Endstop& endstop) const;
    bool isEndstopTriggered(enum AxisEnum axis, int direction) const;
    void acknowledgeEndstopTriggered(const Endstop& endstop);
    void acknowledgeEndstopTriggered(enum AxisEnum axis, int direction);

    inline bool hasUnreportedEndstopHits() const;
    void reportEndstopHits(void (*reportHit)(const Endstop& endstop, long triggeringStep));

    // Step handlers
    inline void onSteppingInX(int direction, long currentStep, bool& triggered);
    inline void onSteppingInY(int direction, long currentStep, bool& triggered);
    inline void onSteppingInZ(int direction, long currentStep, bool& triggered);

  private:
    volatile bool m_ignoreToolSwitch = true;
    volatile bool m_ignoreZSwitch = true;
    volatile bool m_ignoreXYPositionerSwitches = true;

    volatile bool m_hasUnreportedEndstopHits = false;
    volatile long m_triggeringStep;

    Endstops& m_endstops;

    EndstopFilter m_xMin;
    EndstopFilter m_yMin;
    EndstopFilter m_zMax;

    EndstopFilter m_zSwitch;

    EndstopFilter m_xyMinX;
    EndstopFilter m_xyMaxX;
    EndstopFilter m_xyMinY;
    EndstopFilter m_xyMaxY;

    EndstopFilter m_calibrationPlate;

    EndstopFilter m_toolSwitch;

    EndstopFilter* lookup(const Endstop& endstop);

    // ------------------------------------------
    // X-axis

    // Left
    inline void resetLeftEndstops() {
      m_xyMaxX.reset();
    }

    inline void updateLeftEndstops() {
      m_xyMaxX.addSample(READ_PIN(XY_MAX_X));
    }

    inline bool isTriggeredLeft() {
      ScopedInterruptDisable sid;
      return (
        (!m_ignoreXYPositionerSwitches && m_xyMaxX.triggered())
      );
    }

    // Right
    inline void resetRightEndstops() {
      m_xMin.reset();
      m_xyMinX.reset();
    }

    inline void updateRightEndstops() {
      m_xMin.addSample(READ_PIN(X_MIN));
      m_xyMinX.addSample(READ_PIN(XY_MIN_X))
    }

    inline bool isTriggeredRight() {
      ScopedInterruptDisable sid;
      return (
        m_xMin.triggered() ||
        (!m_ignoreXYPositionerSwitches && m_xyMinX.triggered())
      );
    }

    // ------------------------------------------
    // Y-axis

    // Forward
    inline void resetForwardEndstops() {
      m_xyMaxY.reset();
    }

    inline void updateForwardEndstops() {
      m_xyMaxY.addSample(READ_PIN(XY_MAX_Y));
    }

    inline bool isTriggeredForward() {
      ScopedInterruptDisable sid;
      return (
        (!m_ignoreXYPositionerSwitches && m_xyMaxY.triggered())
      );
    }

    // Back
    inline void resetBackEndstops() {
      m_yMin.reset();
      m_xyMinY.reset();
    }

    inline void updateBackEndstops() {
      m_yMin.addSample(READ_PIN(X_MIN));
      m_xyMinY.addSample(READ_PIN(XY_MIN_Y))
    }

    inline bool isTriggeredBack() {
      ScopedInterruptDisable sid;
      return (
        m_yMin.triggered() ||
        (!m_ignoreXYPositionerSwitches && m_xyMinY.triggered())
      );
    }

    // ------------------------------------------
    // Z-axis

    // Up
    inline void resetUpEndstops() {
      m_zMax.reset();
    }

    inline void updateUpEndstops() {
      m_zMax.addSample(READ_PIN(XY_MAX_Y));
    }

    inline bool isTriggeredUp() {
      return m_zMax.triggered();
    }

    // Down
    inline void resetDownEndstops() {
      m_zSwitch.reset();
      m_calibrationPlate.reset();
      m_toolSwitch.reset();
    }

    inline void updateDownEndstops() {
      // Update down endstops
      m_zSwitch.addSample(READ_PIN(Z_MIN));
      m_calibrationPlate.addSample(READ_PIN(P_BOT));

      if (m_ignoreToolSwitch) {
        m_toolSwitch.reset();
      } else {
        bool value = false;
        if (m_endstops.ptop.readDigitalValue(value)) {
          // Treat read failures as TRIGGERED, so that motion stops
          // Note: not sure if this is the right thing to do, but it's
          // safer than allowing motion to continue. Also, if reads are
          // failing there is a deeper bug
          value = true;
        }
        m_toolSwitch.addSample(value);
      }
    }

    inline bool isTriggeredDown() {
      return (
        (!m_ignoreZSwitch && m_zSwitch.triggered()) ||
        (!m_ignoreXYPositionerSwitches && m_calibrationPlate.triggered()) ||
        (!m_ignoreToolSwitch && m_toolSwitch.triggered())
      );
    }
};

bool EndstopMonitor::hasUnreportedEndstopHits() const {
  ScopedInterruptDisable sid;
  return m_hasUnreportedEndstopHits;
}

void EndstopMonitor::onSteppingInX(int direction, long currentStep, bool& triggered) {
  if (direction == -1) {
    // Stepping Right
    resetLeftEndstops();
    updateRightEndstops();
    triggered = isTriggeredRight();
  } else {
    // Stepping Left
    resetRightEndstops();
    updateLeftEndstops();
    triggered = isTriggeredLeft();
  }

  if (triggered && !m_hasUnreportedEndstopHits) {
    m_hasUnreportedEndstopHits = true;
    m_triggeringStep[X_AXIS] = currentStep;
  }
}

void EndstopMonitor::onSteppingInY(int direction, long currentStep, bool& triggered) {
  if (direction == -1) {
    // Stepping Back
    resetForwardEndstops();
    updateBackEndstops()
    triggered = isTriggeredBack();
  } else {
    // Stepping Forward
    resetBackEndstops();
    updateForwardEndstops()
    triggered = isTriggeredForward();
  }

  if (triggered) {
    m_triggeringStep[Y_AXIS] = currentStep;
    m_hasUnreportedEndstopHits = true;
  }
}

void EndstopMonitor::onSteppingInZ(int direction, long currentStep, bool& triggered) {
  if (direction == -1) {
    // Stepping Down
    resetUpEndstops();
    updateDownEndstops()
    triggered = isTriggeredDown();
  } else {
    // Stepping Up
    resetDownEndstops();
    updateUpEndstops()
    triggered = isTriggeredUp();
  }

  if (triggered) {
    m_triggeringStep[Z_AXIS] = currentStep;
    m_hasUnreportedEndstopHits = true;
  }
}
