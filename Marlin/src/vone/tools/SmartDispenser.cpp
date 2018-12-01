#include "SmartDispenser.h"

#include "../../../Marlin.h" // Z_AXIS
#include "../../api/api.h"

tools::SmartDispenser::SmartDispenser(Stepper& stepper)
  : tools::Dispenser(stepper)
{
}
