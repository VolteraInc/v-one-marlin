#pragma once
#include "Dispenser.h"

namespace tools {

class SmartDispenser : public Dispenser {
  public:
    SmartDispenser(Stepper& stepper);

};

}
