#pragma once

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

  private:
    volatile bool m_stopped = false;
};

void Stepper::isr() {
  stepper_isr(endstopMonitor);
}
