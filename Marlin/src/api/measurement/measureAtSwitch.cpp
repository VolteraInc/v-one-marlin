#include "measurement.h"

#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../movement/movement.h"
#include "../../vone/endstops/Endstop.h"

int measureAtSwitch(const Endstop& endstop, float maxTravel, float& measurement) {
  const auto axis = endstop.axis;
  log
    << F("Measure at switch: ")
    << endstop.name
    << endl;

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Move to limit
  if (moveToEndstop(endstop, useDefaultFeedrate, maxTravel) != 0) {
    logError
      << F("Unable to measure at ")
      << endstop.name
      << F(", switch did not trigger during initial approach")
      << endl;
    return -1;
  }
  const float triggerPos = current_position[axis];

  // Retract slightly
  if (retractFromSwitch(endstop)) {
    return -1;
  }
  const float retractPos = current_position[axis];
  const float retractDistance = abs(retractPos - triggerPos);

  // Approach again, slowly
  // NOTE: this gives us a more accurate reading
  const auto slow = homing_feedrate[axis] / 6;
  if (moveToEndstop(endstop, slow, 2 * retractDistance)) {
    logError
      << F("Unable to measure at ") << endstop.name
      << F(" switch, switch did not trigger during second approach")
      << endl;
    return -1;
  }

  // Record the measurement
  measurement = current_position[axis];
  log << F("Measurement: ") << measurement << endl;

  return 0;
}
