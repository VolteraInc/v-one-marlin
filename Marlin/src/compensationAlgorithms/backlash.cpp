#include "../../Marlin.h" // for logging

long applyBacklashCompensation(const char* label, int direction, long steps, long offset) {
  long compensatedSteps = steps + (direction * offset);

  if (logging_enabled) {
    log
      << F("Applying backlash compensation to ") << label << F(": ")
      << steps
      << F(" + ")
      << direction
      << F(" * ")
      << offset
      << F(" -> ")
      << compensatedSteps
      << endl;
  }

  return compensatedSteps;
}
