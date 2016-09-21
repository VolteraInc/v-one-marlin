#include "../Marlin.h" // for logging

float applyBacklashCompensation(int previousDirection, int newDirection, float coordinate, float offset) {

  // backlash compensation is only applied when there is a change in direction
  if (previousDirection == newDirection) {
    return coordinate;
  }

  float adjustedCoordinate = coordinate + (newDirection * offset);

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Applying backlash compensation: ");
    SERIAL_ECHO(coordinate);
    SERIAL_ECHOPGM(" -> ");
    SERIAL_ECHOLN(adjustedCoordinate);
  }

  return adjustedCoordinate;
}
