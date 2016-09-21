#include "../Marlin.h" // for logging

long applyBacklashCompensation(int direction, long steps, long offset) {
  long compensatedSteps = steps + (direction * offset);

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Applying backlash compensation: ");
    SERIAL_ECHO(steps);
    SERIAL_ECHOPGM(" + ");
    SERIAL_ECHO(direction);
    SERIAL_ECHOPGM(" * ");
    SERIAL_ECHO(offset);
    SERIAL_ECHOPGM(" -> ");
    SERIAL_ECHOLN(compensatedSteps);
  }

  return compensatedSteps;
}
