#include "api.h"

#include "../Marlin.h"
#include "../stepper.h"

int measureCalibrationPlateZ(float& plateZ) {
  enable_calibration_plate(true);
  int returnValue = measureAtSwitch(Z_AXIS, -1, useDefaultMaxTravel, plateZ);
  enable_calibration_plate(false);
  return returnValue;
}

int measureProbeDisplacement(float& displacement) {
  // Measure the calibration plate
  float plateZ;
  if (measureCalibrationPlateZ(plateZ)) {
    return -1;
  }

  // Retract
  if (retractFromSwitch(Z_AXIS, -1)) {
    return -1;
  }

  // Probe the calibration plate
  // Note: we do not use the standard probe function here because
  // it would subtract the previously measured displacement (if one exists)
  float probeContactZ;
  if(measureAtSwitch(Z_AXIS, -1, useDefaultMaxTravel, probeContactZ)) {
    return -1;
  }

  displacement = plateZ - probeContactZ;

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Measure probe displacement of "); SERIAL_ECHO(displacement);
    SERIAL_ECHOPGM(" (plateZ="); SERIAL_ECHO(plateZ);
    SERIAL_ECHOPGM(" probeContactZ="); SERIAL_ECHO(probeContactZ);
    SERIAL_ECHOPGM(")\n");
  }


#if VOLTERA_PIN_VERSION == 1
  displacement = 0.050f;
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("This printer does not have a calibration plate, reverting to constant probe displacement of: "); SERIAL_ECHO(displacement);
  SERIAL_ECHOPGM("\n");
#endif

  if (displacement < 0.050f || displacement > 0.500f) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to measure probe displacement, measured value "); SERIAL_ERROR(displacement);
    SERIAL_ERRORPGM(" is outside of the expected range");
    SERIAL_ERRORPGM("\n");
    return -1;
  }

  return 0;
}
