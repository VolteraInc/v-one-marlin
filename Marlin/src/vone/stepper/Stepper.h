#pragma once

class Endstop;
class EndstopMonitor;

class Stepper {
  public:
    Stepper(EndstopMonitor& endstopMonitor);

    bool stopped() const { return m_stopped; }
    void stop();
    void resume();

    int add(float x, float y, float z, float e, float f);

    bool isEndstopTrippered(const Endstop& endstop) const;
    void resetEndstop(const Endstop& endstop);

    inline void isr();

  private:
    volatile bool m_stopped = false;
    EndstopMonitor& m_endstopMonitor;
};

void Stepper::isr() {
  stepper_isr(m_endstopMonitor);
}