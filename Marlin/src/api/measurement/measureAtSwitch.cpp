#include "measurement.h"

#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../movement/movement.h"
#include "../../vone/endstops/Endstop.h"
#include <float.h>

/** This function should not retract after its final measurement. */
int measureAtSwitch(
  const Endstop& endstop,
  float maxTravel,
  float& measurement,
  bool forceConsistency,
  float measurementFeedrate
) {
  const auto axis = endstop.axis;
  log
    << F("Measure at switch: ")
    << endstop.name
    << endl;

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  float lastMeasurement = FLT_MAX;
  float travel = maxTravel;

  for (int i = 0; i < maxTouchCount; ++i) {
    // when pass in -1 for feedrate (useDefaultFeedrate is -1), the moveToEndtop uses homeing_feedrate
    if (moveToEndstop(endstop, measurementFeedrate, travel)) {
      logError
        << F("Unable to measure at ") << endstop.name
        << F(" switch, switch did not trigger during approach")
        << endl;
      return -1;
    }

    const float currentMeasurement = current_position[axis];
    const bool measurementsAreConsistent =
      lastMeasurement != FLT_MAX &&
      abs(lastMeasurement - currentMeasurement) < touchTolerance;

    if (!forceConsistency || measurementsAreConsistent) {
      measurement = lastMeasurement == FLT_MAX
        ? currentMeasurement
        : (lastMeasurement + currentMeasurement) / 2;
      log << F("Measurement: ") << measurement << endl;
      return 0;
    }

    lastMeasurement = currentMeasurement;
    const float triggerPosition = current_position[axis];
    if (retractFromSwitch(endstop)) {
      return -1;
    }
    const float retractDistance = abs(current_position[axis] - triggerPosition);
    travel = 2 * retractDistance;
  }

  logError
    << F("Unable to obtain consistent measurements for ") << endstop.name
    << endl;
  return -1;
}
