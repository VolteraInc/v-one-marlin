#pragma once

class Stepper {
  public:
    Stepper();

    bool stopped() const { return m_stopped; }
    void stop();
    void resume();

    int add(float x, float y, float z, float e, float f);

  private:
    volatile bool m_stopped = false;
};
