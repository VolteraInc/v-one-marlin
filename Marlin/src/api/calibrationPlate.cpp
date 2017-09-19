#include "api.h"

#include "../../Marlin.h"
#include "../../stepper.h"

static const float MinDisplacement = 0.050f;
static const float MaxDisplacement = 0.500f;

static int s_measureCalibrationPlateZ(float& plateZ, float maxTravel) {
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Measuring calibration plate");

  enable_calibration_plate(true);
  int returnValue = measureAtSwitch(Z_AXIS, -1, maxTravel, plateZ);
  enable_calibration_plate(false);

  return returnValue;
}

int measureProbeDisplacement(Tool tool, float& o_displacement) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Measuring probe displacement");
  }

  if(tool != TOOLS_PROBE) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to measure probe displacement, probe not attached");
    return -1;
  }

  // Ensure we are at the correct location
  if (moveToXyPositioner(tool)) {
    return -1;
  }

  // Measure the calibration plate
  float plateZ;
  float maxTravel = 2; // mm, should be within 2mm of the plate
  if (s_measureCalibrationPlateZ(plateZ, maxTravel)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to measure probe displacement, the calibration plate could not be measured (measurement 1 of 2)");
    return -1;
  }

  // Retract
  if (retractFromSwitch(Z_AXIS, -1)) {  ///TODO: not needed ?
    return -1;
  }

  // Probe the calibration plate
  // Note: we do not use the standard probe function here because
  // it would subtract the previously measured displacement (if one exists)
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Measuring the triggering positon of the probe");
  float probeContactZ;
  if(measureAtSwitch(Z_AXIS, -1, MaxDisplacement + Z_HOME_RETRACT_MM, probeContactZ)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to measure probe displacement, the triggering positon of the probe could not be measured (measurement 2 of 2)");
    return -1;
  }

  float displacement = max(0, plateZ - probeContactZ);

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Measuring probe displacement of "); SERIAL_ECHO(displacement);
  SERIAL_ECHOPGM(" (plateZ="); SERIAL_ECHO(plateZ);
  SERIAL_ECHOPGM(" probeContactZ="); SERIAL_ECHO(probeContactZ);
  SERIAL_ECHOPGM(")\n");

  if (displacement < MinDisplacement || displacement > MaxDisplacement) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to measure probe displacement, measured value "); SERIAL_ERROR(displacement);
    SERIAL_ERRORPGM(" is outside of the expected range");
    SERIAL_ERRORPGM("\n");
    return -1;
  }

  o_displacement = displacement;
  return 0;
}
