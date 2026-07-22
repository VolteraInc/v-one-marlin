#include "diagnostics.h"

#include "../api.h"
#include "../../../Marlin.h"
#include "../../vone/endstops/Endstop.h"

#include <math.h>

int measureAtSwitchForDiagnostic(
  const Endstop& endstop,
  float maxTravel,
  float& measurement
) {
  const auto axis = endstop.axis;

  // Ensure the fast approach is a real switch contact, even when a previous
  // measurement intentionally left the switch triggered.
  if (endstop.readTriggered() && retractFromSwitch(endstop)) {
    return -1;
  }

  float fastMeasurement;
  if (measureAtSwitch(endstop, maxTravel, fastMeasurement)) {
    return -1;
  }

  const float fastTriggerPosition = current_position[axis];
  if (retractFromSwitch(endstop)) {
    return -1;
  }
  const float retractDistance = abs(current_position[axis] - fastTriggerPosition);

  const float slowFeedrate = homing_feedrate[axis] / 6;
  float slowMeasurement;
  if (measureAtSwitch(
    endstop,
    2 * retractDistance,
    slowMeasurement,
    true,
    slowFeedrate
  )) {
    return -1;
  }

  const float delta = fabs(slowMeasurement - fastMeasurement);
  log
    << F("Measure at switch fast/slow comparison: ") << endstop.name
    << F(", fast=") << fastMeasurement
    << F(", slow=") << slowMeasurement
    << F(", delta=") << delta
    << endl;

  // Diagnostics use the consistent slow measurement for all calculations.
  measurement = slowMeasurement;
  return 0;
}
