#include "SmartDispenser.h"

#include "../../../Marlin.h" // Z_AXIS
#include "../../api/api.h"

tools::SmartDispenser::SmartDispenser(Stepper& stepper)
  : tools::Dispenser(stepper)
{
}

// DEFER Autopriming
// int primeE(float retract_amount) {
//   // Autopriming procedure for E.
//   // 1. Advance Gear until ink pressure causes a stall.
//   // 2. Retract Gear a small amount to relieve pressure.
//
//   // Enable stallguard.
//   trinamicSetCoolstepMinSpeed(E_AXIS, E_COOLSTEP_MIN_SPEED);
//
//   enable_e_max_endstops(true);
//   log << F("Advancing gear until pressure detected") << endl;
//   if (moveToLimitE(1) != 0) {
//     logError
//       << F("Unable to prime E axis, ink pressure not detected.")
//       << endl;
//       goto DONE;
//   }
//
//   // As soon as we contact the ink , back off to relieve pressure.
//   enable_e_max_endstops(false); // Disable endstops to prevent false triggering.
//   retractFromSwitch(E_AXIS, -1, retract_amount); // Back off the kick amount roughly.
//
// DONE:
//   // Cleanup
//   enable_e_max_endstops(false); // Ensure we exit with endstops disabled.
//   trinamicSetCoolstepMinSpeed(E_AXIS, 0); // Disable stallguard.
//   return 0;
// }
