#pragma once
#include "Tool.h"

namespace tools {

class Dispenser : public Tool {
  public:
    Dispenser(Stepper& stepper);

    int setDispenseHeight(float height);
    float dispenseHeight() const;

    // Additional movement method, used to apply dispense height
    int enqueueDispense(float x, float y, float z, float e, float f);

  private:
    float m_dispenseHeight = 0.0f;

    int meshGears();

    virtual const char* name() const override { return "Dispenser"; }
    virtual int prepareToMoveImpl_Start() override;
    virtual int prepareToMoveImpl_HomeXY() override;
    virtual int prepareToMoveImpl_CalibrateXYZ() override;
    virtual int resetPreparationsImpl() override;
    virtual int enqueueMove(float x, float y, float z, float e, float f) override;
};

}
