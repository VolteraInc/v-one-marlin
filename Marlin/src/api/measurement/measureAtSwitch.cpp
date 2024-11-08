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

  float last_measurement = MAXFLOAT;
  float retractDistance = useDefaultRetractDistance;
  float retraction = retractDistance;
  const auto slow = homing_feedrate[axis] / 6;

  for (int i = 0; i < maxTouchCount; i++){
    //Reset initial position before the each advance iteration;
    //Each retract should retract either the retract distance or the distance moved before the "false" trigger.
    float initialPosition = current_position[axis];

    //have we taken a measurement?  then we should be close -- otherwise travel the full stroke.
    float travel =  last_measurement != MAXFLOAT ? 2 * retractDistance : maxTravel;

    // Approach again, slowly
    // NOTE: this gives us a more accurate reading
    if (moveToEndstop(endstop, slow, travel)) {
      logError
        << F("Unable to measure at ") << endstop.name
        << F(" switch, switch did not trigger during second approach")
        << endl;
      return -1;
    }

    float current_measurement = current_position[axis];
    
    retraction =  min(retractDistance, abs(measurements[i % 2] - initialPosition));
    if (retractFromSwitch(endstop, retraction)) {
      return -1;
    }

    if(last_measurement === MAXFLOAT){
      //always collect at least 2 measurements.
      last_measurement = current_measurement;
      continue;
    }

    //if our measurements are consistent(within tolerance) or we don't care about consistency, return measurement
    if(!forceConsistency || abs(last_measurement - current_measurement) < touchTolerance){
      // Record the measurement, lets take the average
      measurement = (last_measurement + current_measurement) / 2;
      log << F("Measurement: ") << measurement << endl;
      return 0;
    }

    last_measurement = current_measurement;
  }
  
  logError
    << F("Unable to obtain consistent measurements for ") << endstop.name
    << endl;
  return -1;
}
