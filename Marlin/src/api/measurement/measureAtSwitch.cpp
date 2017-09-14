int measureAtSwitch(int axis, int direction, float maxTravel, float& measurement) {
  int returnValue = -1;

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Measure at switch: "); SERIAL_ECHO(direction < 0 ? '-' : '+'); SERIAL_ECHOLN(axis_codes[axis]);
  }

  // Finish any pending moves (prevents crashes)
  st_synchronize();

  // Disable analog reads
  auto mustRestoreADC = false;
  if (axis == Z_AXIS && direction == -1 && periodicAnalogReadsEnabled()) {
    mustRestoreADC = true;
    set_p_top_mode(P_TOP_COMMS_READ_MODE);
  }

  // Move to limit
  if (moveToLimit(axis, direction, useDefaultFeedrate, maxTravel) != 0) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to measure at "); SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
    SERIAL_ERRORPGM(" switch, switch did not trigger during initial approach\n");
    goto DONE;
  }

  // Retract slightly
  if (retractFromSwitch(axis, direction)) {
    goto DONE;
  }

  // Approach again, slowly
  // NOTE: this gives us a more accurate reading
  {
    const auto slow = homing_feedrate[axis] / 6;
    const float retractDistance = s_defaultRetractDistance[axis];
    if (moveToLimit(axis, direction, slow, 2 * retractDistance)) {
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to measure at "); SERIAL_ERROR(direction < 0 ? '-' : '+'); SERIAL_ERROR(axis_codes[axis]);
      SERIAL_ERRORPGM(" switch, switch did not trigger during second approach\n");
      goto DONE;
    }
  }

  // Record the measurement
  measurement = current_position[axis];
  if(logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Measurement: "); SERIAL_ECHOLN(measurement);
  }

  returnValue = 0;

DONE:
  // restore ADC read state
  if (mustRestoreADC) {
    set_p_top_mode(P_TOP_NORMAL_MODE);
  }

  return returnValue;
}
