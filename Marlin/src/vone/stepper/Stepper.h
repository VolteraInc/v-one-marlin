#pragma once

#include "../../../MarlinConfig.h"
#include "../../utils/HighwaterReporter.h"

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

    int add(float x, float y, float z, float e, float f);

    // DEFER: Ideally the stepper isr would be defined in this file
    //        but carving stepper_isr() into multiple h-files +
    //        classes to hold shared variables is too much work for
    //        right now

    // Monitoring/Reporting
    HighwaterReporter maxStepsComplete;
    HighwaterReporter maxInterruptsAllowed;
    HighwaterReporter maxCompletedWithoutTriggers;
    HighwaterReporter maxCompletedWithoutTriggersTics;
    HighwaterReporter maxStepRate;
    HighwaterReporter maxStepTiming;
    void periodicReport();
    void outputStatus();

  private:
    const __FlashStringHelper* volatile m_stopReason = nullptr;

    const __FlashStringHelper* stopReason() const;
};
