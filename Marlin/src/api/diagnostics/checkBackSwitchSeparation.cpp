#include "../../api/api.h"
#include "../../../Marlin.h"
#include "../../../ConfigurationStore.h"
#include "../../vone/VOne.h"
#include "../../vone/tools/Probe.h"

static const float MinDistance = 0.500;

int s_measure(const Endstop& endstop, float measurements[], unsigned int numMeasurements) {
  for (auto i = 0u; i < numMeasurements; ++i) {
    if (measureAtSwitch(endstop, 10.0, measurements[i])) {
      return -1;
    }
  }
  return 0;
}

#ifndef XYZ_STRAIN //with new strain guage design, the XYZ positoner is brought significantly forward, so this test is not required

// Confirm the minY and xyMinY switches are far enough apart.
// If these switches are too close it's possible we'll hit the
// minY switch instead of the xyMinY switch
static int s_checkBackSwitchSeparation(tools::Tool& tool) {
  const auto& xyPositionerBack = vone->endstops.xyPositionerBack;
  const auto& yMin = vone->endstops.yMin;

  static const size_t NUM_MEASUREMENTS = 3;
  bool pass = true;
  float distances[NUM_MEASUREMENTS];
  struct {
    float xyBack[NUM_MEASUREMENTS];
    float yMin[NUM_MEASUREMENTS];
  } measurements;

  if (
    tool.prepareToMove() ||
    raiseToSoftMax(tool) ||

    // Measure xyPositionerBack
    moveToXyPositioner(tool) ||
    s_measure(xyPositionerBack, measurements.xyBack, NUM_MEASUREMENTS) ||

    // Exit the xy-positioner
    // Note: retract from the switch first rather than raising while still
    //       in contact with xyPositionerBack
    retractFromSwitch(xyPositionerBack) ||
    raiseToSoftMax(tool) ||

    // Measure yMin
    s_measure(yMin, measurements.yMin, NUM_MEASUREMENTS) ||

    // Retract from yMin, rather than staying in contact with it
    // Note: We could go home here (we use to) but this would
    //       leave the carriage in the home position if the
    //       separation test fails. Whereas, staying near the
    //       xy-positioner helps to indicate which test failed
    //       (in the i2 sequence).
    retractFromSwitch(yMin)
  ) {
    return -1;
  }

  // Compute distances
  for (auto i = 0u; i < NUM_MEASUREMENTS; ++i) {
    distances[i] = measurements.xyBack[i] - measurements.yMin[i];
    if (distances[i] < MinDistance) {
      pass = false;
    }
  }

  // Log data
  log
    << F("check back switch separation measurements = [")
    << ArrayWithSize<float>(distances, NUM_MEASUREMENTS)
    << F("] xyPositionerBack = [")
    << ArrayWithSize<float>(measurements.xyBack, NUM_MEASUREMENTS)
    << F("] yMin = [")
    << ArrayWithSize<float>(measurements.yMin, NUM_MEASUREMENTS)
    << F("]")
    << endl;

  // Report if failed
  if (!pass) {
    logError
      << F("Unable to complete back switch separation check, distance from ")
      << yMin.name << F(" to ") << xyPositionerBack.name
      << F(" was less than ")
      << FloatWithFormat(MinDistance, 3)
      << F("mm, measurements = [")
      << ArrayWithSize<float>(distances, NUM_MEASUREMENTS)
      << F("]")
      << endl;
    return -1;
  }

  // Success
  return 0;
}

int checkBackSwitchSeparation(tools::Tool& tool) {
  int returnValue = s_checkBackSwitchSeparation(tool);

  // Note: Touching switches and resycning with the stepper
  //       can introduce a small error, we don't want that
  //       error impacting any commands run after this one.
  setHomedState(X_AXIS, 0);
  setHomedState(Y_AXIS, 0);
  setHomedState(Z_AXIS, 0);
  return (
    tool.resetPreparations() ||
    returnValue
  );
}

#endif
