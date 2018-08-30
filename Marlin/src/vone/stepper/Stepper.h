#pragma once

#include "../../../MarlinConfig.h"

class Endstop;
class EndstopMonitor;

class Stepper {
  public:
    Stepper(EndstopMonitor& endstopMonitor);

    EndstopMonitor& endstopMonitor;

    bool stopped() const { return m_stopped; }
    void stop();
    void resume();

    int add(float x, float y, float z, float e, float f);

    unsigned long maxStepperDurationMicros();

    // DEFER: Ideally the stepper isr would be defined in this file
    //        but carving stepper_isr() into multiple h-files +
    //        classes to hold shared variables is too much work for
    //        right now

  private:
    volatile bool m_stopped = false;
    volatile unsigned long m_maxStepperDurationMicros = 0;
};
