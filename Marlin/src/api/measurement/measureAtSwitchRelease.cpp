#include "measurement.h"

#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../../../planner.h"
#include "../../utils/rawToVoltage.h"
#include "../tools/tools.h"
#include "../movement/movement.h"

static unsigned s_countTriggers(unsigned maxSamples) {
  if (logging_enabled) {
  }

  auto count = 0u;
  float voltages[maxSamples];
  for (auto i = 0u; i < maxSamples; ++i) {
    // HACK: use probe function since we only use this function for probing right now.
    // if we ever use it for other axes we can figure out a solution then
    // e.g. pass function, use templates, or look into SmallFun
    if (Probe::readAnalogTriggered(&voltages[i])) {
      ++count;
    }
  }

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("countTriggers voltages: ["); serialArray(voltages, maxSamples);
    SERIAL_PAIR("], count: ", count);
    SERIAL_PAIR(", position: ", current_position[Z_AXIS]);
    SERIAL_EOL;
  }

  return count;
}

// TODO int analogMeasureAtSwitch(int axis, int direction, unsigned pin, float initialApporachSpeed, float& digitallyTriggeredAt, float& fullyTriggeredAt, float& releaseStartedAt, float& releaseCompletedAt) {
//   // Might based this on measureAtSwitchRelease, but allow back off too
//
//   // Disable digital trigger detection
//
//   return -1;
// }

int measureAtSwitchRelease(int axis, int direction, float& releaseStartedAt, float& releaseCompletedAt, unsigned delay_ms) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Measure at switch retract: ");
    SERIAL_ECHO(direction < 0 ? '-' : '+'); SERIAL_ECHOLN(axis_codes[axis]);
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Note: 1mm should be more than enough for a release
  const auto maxTravel = 1u;
  const auto maxSteps = maxTravel * axis_steps_per_unit[axis];
  const auto numSamples = 20u;
  bool releaseStarted = false;
  const float distance = 1 / axis_steps_per_unit[axis];
  for (auto i = 0u; i < maxSteps; ++i) {
    // Examine triggered status
    const auto count = s_countTriggers(numSamples);
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
    if(logging_enabled) {
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("Retract by: "); SERIAL_ECHOLN(distance);
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

  SERIAL_ERROR_START;
  SERIAL_ERRORPGM("Unable to measure at release of ");
  SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
  SERIAL_PAIR(" switch, switch did not release after ", maxTravel);
  SERIAL_ERROR("mm of travel");
  SERIAL_EOL;

  return -1;
}
