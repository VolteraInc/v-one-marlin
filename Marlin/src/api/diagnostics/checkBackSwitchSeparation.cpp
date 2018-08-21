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

// Confirm the minY and xyMinY switches are far enough apart.
// If these switches are too close it's possible we'll hit the
// minY switch instead of the xyMinY switch
int checkBackSwitchSeparation(tools::Tool& tool) {
  int returnValue = -1;
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
    raise() ||

    // Measure xyPositionerBack
    moveToXyPositioner(tool) ||
    s_measure(xyPositionerBack, measurements.xyBack, NUM_MEASUREMENTS) ||

    // Measure yMin
    raise() ||
    s_measure(yMin, measurements.yMin, NUM_MEASUREMENTS)
  ) {
    goto DONE;
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
      << F("Unable complete back switch separation check, distance from ")
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
  returnValue = 0;

DONE:
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
