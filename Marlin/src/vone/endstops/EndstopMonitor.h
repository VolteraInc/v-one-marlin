#pragma once

#include "../../../Axis.h"
#include "../../utils/ScopedInterruptDisable.h"
#include "../../utils/ScopedInterruptDisable.h"
#include "../pins/PTopPin/PTopPin.h"
#include "Endstops.h"
#include "EndstopFilter.h"
#include "TriggerEventLog.h"

class Endstop;
float stepsToPositionInAxis(AxisEnum axis, volatile long stepCounts[NUM_AXIS]);

class EndstopMonitor {
  public:
    EndstopMonitor(Endstops& endstops);

    void ignore(const Endstop& endstop, bool ignore = true);
    bool ignoring(const Endstop& endstop) const;

    bool isTriggered(const Endstop& endstop) const;
    bool isTriggered(enum AxisEnum axis) const;
    int acknowledgeTriggered(const Endstop& endstop);
    int acknowledgeTriggered(enum AxisEnum axis, int direction);

    FORCE_INLINE bool hasUnreportedHits() const;
    void reportHits(void (*reportHit)(const Endstop& endstop, float triggeringPosition));

    // Step handlers
    FORCE_INLINE void onSteppingInX(int direction, volatile long stepCounts[NUM_AXIS], bool& triggered);
    FORCE_INLINE void onSteppingInY(int direction, volatile long stepCounts[NUM_AXIS], bool& triggered);
    FORCE_INLINE void onSteppingInZ(int direction, volatile long stepCounts[NUM_AXIS], bool& triggered);

  private:
    TriggerEventLog m_triggerLog;
    volatile long m_triggeringStep;

    Endstops& m_endstops;

    EndstopFilter m_xMin;
    EndstopFilter m_yMin;
    EndstopFilter m_zMax;

    EndstopFilter m_zSwitch;

    EndstopFilter m_xyPositionerLeft;
    EndstopFilter m_xyPositionerRight;
    EndstopFilter m_xyPositionerForward;
    EndstopFilter m_xyPositionerBack;
    EndstopFilter m_calibrationPlate;

    EndstopFilter m_toolSwitch;

    const EndstopFilter* lookup(const Endstop& endstop) const;
    EndstopFilter* lookup(const Endstop& endstop);

    // ------------------------------------------
    // X-axis

    // Left
    FORCE_INLINE void resetLeftEndstops() {
      m_xyPositionerLeft.reset();
    }

    FORCE_INLINE void updateEndstop(
      EndstopFilter& filter,
      bool value,
      const Endstop& endstop,
      volatile long stepCounts[NUM_AXIS]
    ) {
      filter.addSample(value);
      if (filter.triggered()) {
        m_triggerLog.push(endstop, stepsToPositionInAxis(endstop.axis, stepCounts));
      }
    }

    FORCE_INLINE void updateLeftEndstops(volatile long stepCounts[NUM_AXIS]) {
      updateEndstop(m_xyPositionerLeft, READ_PIN(XY_MAX_X), m_endstops.xyPositionerLeft, stepCounts);
    }

    FORCE_INLINE bool isTriggeredLeft() const {
      ScopedInterruptDisable sid;
      return m_xyPositionerLeft.triggered();
    }

    // Right
    FORCE_INLINE void resetRightEndstops() {
      m_xMin.reset();
      m_xyPositionerRight.reset();
    }

    FORCE_INLINE void updateRightEndstops(volatile long stepCounts[NUM_AXIS]) {
      updateEndstop(m_xMin, READ_PIN(X_MIN), m_endstops.xMin, stepCounts);
      updateEndstop(m_xyPositionerRight, READ_PIN(XY_MIN_X), m_endstops.xyPositionerRight, stepCounts);
    }

    FORCE_INLINE bool isTriggeredRight() const {
      ScopedInterruptDisable sid;
      return m_xMin.triggered() || m_xyPositionerRight.triggered();
    }

    // ------------------------------------------
    // Y-axis

    // Forward
    FORCE_INLINE void resetForwardEndstops() {
      m_xyPositionerForward.reset();
    }

    FORCE_INLINE void updateForwardEndstops(volatile long stepCounts[NUM_AXIS]) {
      updateEndstop(m_xyPositionerForward, READ_PIN(XY_MAX_Y), m_endstops.xyPositionerForward, stepCounts);
    }

    FORCE_INLINE bool isTriggeredForward() const {
      ScopedInterruptDisable sid;
      return m_xyPositionerForward.triggered();
    }

    // Back
    FORCE_INLINE void resetBackEndstops() {
      m_yMin.reset();
      m_xyPositionerBack.reset();
    }

    FORCE_INLINE void updateBackEndstops(volatile long stepCounts[NUM_AXIS]) {
      updateEndstop(m_yMin, READ_PIN(Y_MIN), m_endstops.yMin, stepCounts);
      updateEndstop(m_xyPositionerBack, READ_PIN(XY_MIN_Y), m_endstops.xyPositionerBack, stepCounts);
    }

    FORCE_INLINE bool isTriggeredBack() const {
      ScopedInterruptDisable sid;
      return m_yMin.triggered() || m_xyPositionerBack.triggered();
    }

    // ------------------------------------------
    // Z-axis

    // Up
    FORCE_INLINE void resetUpEndstops() {
      m_zMax.reset();
    }

    FORCE_INLINE void updateUpEndstops(volatile long stepCounts[NUM_AXIS]) {
      updateEndstop(m_zMax, READ_PIN(Z_MAX), m_endstops.zMax, stepCounts);
    }

    FORCE_INLINE bool isTriggeredUp() const {
      return m_zMax.triggered();
    }

    // Down
    FORCE_INLINE void resetDownEndstops() {
      m_zSwitch.reset();
      m_calibrationPlate.reset();
      m_toolSwitch.reset();
    }

    FORCE_INLINE void updateDownEndstops(volatile long stepCounts[NUM_AXIS]) {
      updateEndstop(m_zSwitch, READ_PIN(Z_MIN), m_endstops.zSwitch, stepCounts);
      updateEndstop(m_calibrationPlate, READ_PIN(P_BOT), m_endstops.calibrationPlate, stepCounts);
      updateEndstop(m_toolSwitch, READ_PIN(P_TOP), m_endstops.toolSwitch, stepCounts);
    }

    FORCE_INLINE bool isTriggeredDown() const {
      return (
        m_zSwitch.triggered() ||
        m_calibrationPlate.triggered() ||
        m_toolSwitch.triggered()
      );
    }
};

bool EndstopMonitor::hasUnreportedHits() const {
  ScopedInterruptDisable sid;
  return !m_triggerLog.empty();
}

void EndstopMonitor::onSteppingInX(int direction, volatile long stepCounts[NUM_AXIS], bool& triggered) {
  if (direction == -1) {
    // Stepping Right
    resetLeftEndstops();
    updateRightEndstops(stepCounts);
    triggered = isTriggeredRight();
  } else {
    // Stepping Left
    resetRightEndstops();
    updateLeftEndstops(stepCounts);
    triggered = isTriggeredLeft();
  }
}

void EndstopMonitor::onSteppingInY(int direction, volatile long stepCounts[NUM_AXIS], bool& triggered) {
  if (direction == -1) {
    // Stepping Back
    resetForwardEndstops();
    updateBackEndstops(stepCounts);
    triggered = isTriggeredBack();
  } else {
    // Stepping Forward
    resetBackEndstops();
    updateForwardEndstops(stepCounts);
    triggered = isTriggeredForward();
  }
}

void EndstopMonitor::onSteppingInZ(int direction, volatile long stepCounts[NUM_AXIS], bool& triggered) {
  if (direction == -1) {
    // Stepping Down
    resetUpEndstops();
    updateDownEndstops(stepCounts);
    triggered = isTriggeredDown();
  } else {
    // Stepping Up
    resetDownEndstops();
    updateUpEndstops(stepCounts);
    triggered = isTriggeredUp();
  }
}
