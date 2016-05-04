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
  float probeContactZ;
  if(probe(probeContactZ)) {
    return -1;
  }

  displacement = abs(plateZ - probeContactZ);

#if VOLTERA_PIN_VERSION == 1
  displacement = 0.05f;
  SERIAL_ECHO_START;
  SERIAL_ECHO("This printer does not have a calibration plate, reverting to constant probe displacement of: ");
  SERIAL_ECHO(displacement);
  SERIAL_ECHO("\n");
#endif

  if (displacement < 0.050f || displacement > 0.550f) {
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to measure probe displacement, measured value ");
    SERIAL_ERROR(displacement);
    SERIAL_ERROR(" is outside of the expected range");
    SERIAL_ERROR("\n");
    return -1;
  }

  return 0;
}
