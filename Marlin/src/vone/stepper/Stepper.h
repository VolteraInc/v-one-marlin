#pragma once

class Stepper {
  public:
    Stepper();

    void stop();
    void resume();

    int add(float x, float y, float z, float e, float f);

  private:
    volatile bool m_stopped;
};
