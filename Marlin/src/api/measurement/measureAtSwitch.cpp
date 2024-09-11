#include "measurement.h"

#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../movement/movement.h"
#include "../../vone/endstops/Endstop.h"

int measureAtSwitch(const Endstop& endstop, float maxTravel, float& measurement, bool forceConsistency) {
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
  const float initialTriggerPos = current_position[axis];
  float measurements[2] = {initialTriggerPos, initialTriggerPos};

  for (int i = 0; i < maxTouchCount; i++){
    // Retract slightly
    if (retractFromSwitch(endstop)) {
      return -1;
    }
    const float retractPos = current_position[axis];

    //limit retraction distance to the initial touch-retraction movement, don't let it continuously back off
    const float retractDistance = abs(retractPos - measurements[i % 2]);

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

    //record measurement
    measurements[(i + 1) % 2] = current_position[axis];

    //if our measurements are consistent(within tolerance) or we don't care about consistency, return measurement
    if(!forceConsistency || abs(measurements[0] - measurements[1]) < touchTolerance){
      // Record the measurement, lets take the average
      measurement = (measurements[0] + measurements[1]) / 2;
      log << F("Measurement: ") << measurement << endl;

      return 0;
    }
  }
  
  logError
    << F("Unable to obtain consistent measurements for ") << endstop.name
    << endl;
  return -1;
}
