#include "../Marlin.h" // for logging

float applyBacklashCompensation(int direction, float coordinate, float offset) {

  float adjustedCoordinate = coordinate + (direction * offset);

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Applying backlash compensation: ");
    SERIAL_ECHO(coordinate);
    SERIAL_ECHOPGM(" -> ");
    SERIAL_ECHOLN(adjustedCoordinate);
  }

  return adjustedCoordinate;
}
