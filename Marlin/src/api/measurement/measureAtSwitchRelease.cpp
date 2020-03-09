#include "measurement.h"

#include "../movement/movement.h"
#include "../../../Marlin.h"
#include "../../vone/VOne.h"

static unsigned s_countTriggers(const Endstop& endstop, unsigned maxSamples) {
  const auto startTime = millis();
  auto count = 0u;
  float voltages[maxSamples];

  if (endstop.pin != P_TOP_PIN) {
    logWarning
      << F("Sampling voltage of ")
      << endstop.name
      << F(" is not supported, returning 0")
      << endl;
    return 0;
  }

  for (auto i = 0u; i < maxSamples; ++i) {
    // HACK: use probe function since we only use this function for probing right now.
    // if we ever use it for other axes we can figure out a solution then
    // e.g. pass function, use templates, or look into SmallFun
    if (vone->toolBox.probe.readAnalogTriggered(&voltages[i])) {
      ++count;
    }
  }

  if (logging_enabled) {
    const auto duration = millis() - startTime;
    log
      << F("countTriggers voltages: [") << ArrayWithSize<float>(voltages, maxSamples)
      << F("], duration: ") << duration
      << F(", count: ") << count
      << F(", position: ") << current_position[Z_AXIS]
      << endl;
  }

  return count;
}

static int s_measureAtSwitchRelease(const Endstop& endstop, float& releaseStartedAt, float& releaseCompletedAt, unsigned delay_ms) {
  if (logging_enabled) {
    log << F("Measure at switch release: ") << endstop.name << endl;
  }

  const auto axis = endstop.axis;
  const auto direction = endstop.direction;

  // Finish any pending moves (prevents crashes)
  vone->stepper.finishPendingMoves();

  // Note: 1mm should be more than enough for a release
  const auto maxTravel = 1u;
  const auto maxSteps = millimetersToSteps(maxTravel, axis);
  const auto numSamples = 20u;
  bool releaseStarted = false;
  const float distance = stepsToMillimeters(1, axis);
  for (auto i = 0u; i < maxSteps; ++i) {
    // Examine triggered status
    const auto count = s_countTriggers(endstop, numSamples);
    if (count < numSamples) {
      // Set release start position, if we haven't already
      if (!releaseStarted) {
        // Completely released, record position and exit
        releaseStarted = true;
        releaseStartedAt = current_position[axis];
      }

      // Completely released, record position and exit
      if (count == 0) {
        releaseCompletedAt = current_position[axis];
        return 0;
      }
    }

    // Retract by one step
    if (logging_enabled) {
      log << F("Retract by: ") << distance << endl;
    }
    if (relativeRawMoveXYZ(
        axis == X_AXIS ? distance * -direction : 0,
        axis == Y_AXIS ? distance * -direction : 0,
        axis == Z_AXIS ? distance * -direction : 0
    )) {
      return -1;
    }

    // Delay, to allow voltage to stabilize
    if (delay_ms > 0) {
      delay(delay_ms);
    }
  }

  logError
    << F("Unable to measure at release of ")
    << endstop.name
    << F(", switch did not release after ") << maxTravel
    << F("mm of travel")
    << endl;

  return -1;
}

int measureAtSwitchRelease(const Endstop& endstop, float& releaseStartedAt, float& releaseCompletedAt, unsigned delay_ms) {

  int returnValue = -1;

  // Tell the tool detector that we have started the probing sequence.

  returnValue = s_measureAtSwitchRelease(endstop, releaseStartedAt, releaseCompletedAt, delay_ms);

  // Before returning, tell the tool detector that we have finished the probing sequence.

  return returnValue;
}
