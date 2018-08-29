#pragma once

#include "../../../MarlinConfig.h"
#include "../../../stepper.h"

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

    inline void isr();

    unsigned long maxStepperDurationMicros();

  private:
    volatile bool m_stopped = false;
    volatile unsigned long m_maxStepperDurationMicros = 0;
};

void Stepper::isr() {
  const auto start = micros();

  stepper_isr(endstopMonitor);

  const auto elapsed = micros() - start;
  if (elapsed > m_maxStepperDurationMicros) {
    m_maxStepperDurationMicros = elapsed;
  }
}
