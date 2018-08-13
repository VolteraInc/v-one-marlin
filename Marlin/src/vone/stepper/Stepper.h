#pragma once

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

    // Endstops
    bool isEndstopTriggered(const Endstop& endstop) const;
    bool isEndstopTriggered(AxisEnum axis) const;
    void acknowledgeEndstopTriggered(const Endstop& endstop);
    void acknowledgeEndstopTriggered(AxisEnum axis);
    bool hasUnreportedEndstopHits() const;
    void reportEndstopHits();

    inline void isr();

  private:
    volatile bool m_stopped = false;
};

void Stepper::isr() {

  stepper_isr(endstopMonitor);
}