// DEFER: could pass this as a template param (if we need to in order to generalize beyond p_top)
unsigned countTriggers(unsigned pin, unsigned maxSamples) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHO("countTriggers ");
  }

  auto count = 0u;
  for (auto i = 0u; i < maxSamples; ++i) {
    auto voltage = rawToVoltage(analogRead(pin));

    if (logging_enabled) {
      if (i==0) {
        SERIAL_PAIR("voltages: [", voltage);
      } else {
        SERIAL_PAIR(", ", voltage);
      }
    }

    // HACK: use probe function since we only use this function for probing right now.
    // if we ever use it for other axes we ca figure out a solution then
    if (Probe::isTriggered(voltage)) {
      ++count;
    }
  }

  if (logging_enabled) {
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

int measureAtSwitchRelease(int axis, int direction, unsigned pin, float& releaseStartedAt, float& releaseCompletedAt, unsigned delay_ms) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Measure at switch retract: ");
    SERIAL_ECHO(direction < 0 ? '-' : '+'); SERIAL_ECHOLN(axis_codes[axis]);
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Disable analog reads (prevent false tool change detection)
  PTopScopedUsageLock scopedUse;

  // Note: 1mm should be more than enough for a release
  const auto maxTravel = 1u;
  const auto maxSteps = maxTravel * axis_steps_per_unit[axis];
  const auto numSamples = 20u;
  bool releaseStarted = false;
  const float distance = 1 / axis_steps_per_unit[axis];
  for (auto i = 0u; i < maxSteps; ++i) {
    // Examine triggered status
    const auto count = countTriggers(pin, numSamples);
    if (count < numSamples) {
      // Set release start position, if we haven't already
      if (!releaseStarted) {
        // Completely released, record position and exit
        // if (count == 0) {
          // const float distance = 10 / axis_steps_per_unit[axis];
          // if (s_relativeRawMoveXYZ(
          //     axis == X_AXIS ? distance * -direction : 0,
          //     axis == Y_AXIS ? distance * -direction : 0,
          //     axis == Z_AXIS ? distance * -direction : 0
          // )) {
          //   goto DONE;
          // }
          // continue;
        // } else {
          releaseStarted = true;
          releaseStartedAt = current_position[axis];
        // }
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
    if (s_relativeRawMoveXYZ(
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
