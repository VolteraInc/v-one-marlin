#pragma once
#include "Tool.h"

namespace tools {

class NullTool : public Tool {
  public:
    NullTool(Stepper& stepper);

  private:
    virtual const char* name() const override { return "None"; }
    virtual int prepareToMoveImpl() override;
    virtual int resetPreparationsImpl() override;
    virtual int enqueueMove(float x, float y, float z, float e, float f) override;
};

}