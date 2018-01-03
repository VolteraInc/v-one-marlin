#pragma once
#include "Tool.h"

class PTopPin;

namespace tools {

class Router : public Tool {
  public:
    Router(Stepper& stepper, PTopPin& pin);

    int setRotationSpeed(unsigned speed);
    float rotationSpeed() const { return m_rotationSpeed; }

  private:
    PTopPin& m_pin;
    int m_rotationSpeed = 0;

    int stopRotation();
    int stopRotationIfMounted();

    virtual const char* name() const override { return "Router"; }
    virtual int prepareToMoveImpl() override;
    virtual int resetPreparationsImpl() override;
    virtual int enqueueMove(float x, float y, float z, float e, float f) override;

};

}
