#pragma once
#include "Tool.h"

namespace tools {

class NullTool : public Tool {
  public:
    NullTool(Stepper& stepper);

  private:
    virtual const char* type() const override { return "None"; }
    virtual unsigned int version() const override { return 1; }

    virtual int prepareToMoveImpl_Start() override;
    virtual int prepareToMoveImpl_HomeXY() override;
    virtual int prepareToMoveImpl_CalibrateXYZ() override;
    virtual int resetPreparationsImpl() override;
    virtual int enqueueMove(float x, float y, float z, float e, float f) override;
};

}
