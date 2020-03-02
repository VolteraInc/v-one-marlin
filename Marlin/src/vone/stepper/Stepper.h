#pragma once

#include "../../../MarlinConfig.h"
#include "../../../Axis.h"
#include "../../utils/Reporters.h"

class Endstop;
class EndstopMonitor;

class Stepper {
  public:
    Stepper(EndstopMonitor& endstopMonitor);
    void start();

    EndstopMonitor& endstopMonitor;

    bool stopped() const;
    void stop(const __FlashStringHelper* reason);
    void resume();

    void enableSkewAdjustment(bool enable = true);

    int overrideCurrentPosition(float x, float y, float z, float e);
    int overrideCurrentPosition(float position[4]);
    int overrideCurrentPosition(AxisEnum axis, float value);

    // Override the curent position based on the stepper's position
    // NOTE: Certain movements, like attempting to move past an end-stop, will leave the
    //       planner out of sync with the stepper. This function corrects the planner's position.
    int resyncWithStepCount(AxisEnum axis);
    int resyncWithStepCount(bool x, bool y, bool z, bool e);

    int add(float x, float y, float z, float e, float f);

    void finishPendingMoves() const;

    // DEFER: Ideally the stepper isr would be defined in this file
    //        but carving stepper_isr() into multiple h-files +
    //        classes to hold shared variables is too much work for
    //        right now

    // Monitoring/Reporting
    HighWaterReporter maxStepsComplete;
    HighWaterReporter maxInterruptsAllowed;
    HighWaterReporter maxCompletedWithoutTriggers;
    HighWaterReporter maxCompletedWithoutTriggersTics;
    HighWaterReporter maxStepRate;
    LowWaterReporter maxStepTiming;
    void periodicReport();
    void outputStatus();

  private:
    const __FlashStringHelper* volatile m_stopReason = nullptr;

    const __FlashStringHelper* stopReason() const;
};
