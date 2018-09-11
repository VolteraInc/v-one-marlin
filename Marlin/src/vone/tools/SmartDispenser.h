#pragma once
#include "Dispenser.h"

namespace tools {

class SmartDispenser : public Dispenser {
  public:
    SmartDispenser(Stepper& stepper);

  private:
    virtual const char* name() const override { return "Dispenser"; }
};

}
