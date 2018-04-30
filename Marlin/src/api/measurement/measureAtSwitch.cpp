#include "measurement.h"

#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../movement/movement.h"

int measureAtSwitch(int axis, int direction, float maxTravel, float& measurement) {
  log
    << F("Measure at switch: ")
    << (direction < 0 ? '-' : '+') << axis_codes[axis]
    << endl;

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Move to limit
  if (moveToLimit(axis, direction, useDefaultFeedrate, maxTravel) != 0) {
    logError
      << F("Unable to measure at ")
      << (direction < 0 ? '-' : '+') << axis_codes[axis]
      << F(" switch, switch did not trigger during initial approach")
      << endl;
    return -1;
  }
  const float triggerPos = current_position[axis];

  // Retract slightly
  if (retractFromSwitch(axis, direction)) {
    return -1;
  }
  const float retractDistance = abs(current_position[axis] - triggerPos);

  // TODO: Confirm switch released

  // Approach again, slowly
  // NOTE: this gives us a more accurate reading
  const auto slow = homing_feedrate[axis] / 6;
  if (moveToLimit(axis, direction, slow, 2 * retractDistance)) {
    logError
      << F("Unable to measure at ") << (direction < 0 ? '-' : '+') << axis_codes[axis]
      << F(" switch, switch did not trigger during second approach")
      << endl;
    return -1;
  }

  // Record the measurement
  measurement = current_position[axis];
  log << F("Measurement: ") << measurement << endl;

  return 0;
}
