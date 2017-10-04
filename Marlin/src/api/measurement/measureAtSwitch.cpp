#include "measurement.h"

#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../movement/movement.h"

int measureAtSwitch(int axis, int direction, float maxTravel, float& measurement) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Measure at switch: "); SERIAL_ECHO(direction < 0 ? '-' : '+'); SERIAL_ECHOLN(axis_codes[axis]);
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Move to limit
  if (moveToLimit(axis, direction, useDefaultFeedrate, maxTravel) != 0) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to measure at "); SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERRORPGM(" switch, switch did not trigger during initial approach\n");
    return -1;
  }
  const float triggerPos = current_position[axis];

  // Retract slightly
  if (retractFromSwitch(axis, direction)) {
    return -1;
  }
  const float retractDistance = abs(current_position[axis] - triggerPos);

  // Approach again, slowly
  // NOTE: this gives us a more accurate reading
  const auto slow = homing_feedrate[axis] / 6;
  if (moveToLimit(axis, direction, slow, 2 * retractDistance)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to measure at "); SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERRORPGM(" switch, switch did not trigger during second approach\n");
    return -1;
  }

  // Record the measurement
  measurement = current_position[axis];
  if(logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Measurement: "); SERIAL_ECHOLN(measurement);
  }

  return 0;
}
