#include "api.h"

#include "../../Marlin.h" // Z_AXIS
#include "../../stepper.h" // enable_calibration_plate()
#include "../vone/tools/Probe.h"
#include "../vone/VOne.h"

static const float MinDisplacement = 0.050f;
static const float MaxDisplacement = 0.500f;

static int s_measureCalibrationPlateZ(float& plateZ, float maxTravel) {
  log << F("Measuring calibration plate") << endl;
  auto& endstopMonitor = vone->stepper.endstopMonitor;

  endstopMonitor.ignoreCalibrationPlate(false);
  int returnValue = measureAtSwitch(Z_AXIS, -1, maxTravel, plateZ);
  endstopMonitor.ignoreCalibrationPlate();

  return returnValue;
}

int measureProbeDisplacement(tools::Probe& probe, float& o_displacement) {
  log << F("Measuring probe displacement") << endl;

  if (probe.detached()) {
    logError
      << F("Unable to measure probe displacement, probe not attached")
      << endl;
    return -1;
  }

  // Ensure we are at the correct location
  if (moveToXyPositioner(probe)) {
    return -1;
  }

  // Measure the calibration plate
  float plateZ;
  float maxTravel = 2; // mm, should be within 2mm of the plate
  if (s_measureCalibrationPlateZ(plateZ, maxTravel)) {
    logError
      << F("Unable to measure probe displacement, ")
      << F("the calibration plate could not be measured (measurement 1 of 2)")
      << endl;
    return -1;
  }

  // Retract
  if (retractFromSwitch(Z_AXIS, -1)) {  ///TODO: not needed ?
    return -1;
  }

  // Probe the calibration plate
  // Note: we do not use the standard probe function here because
  // it would subtract the previously measured displacement (if one exists)
  log << F("Measuring the triggering positon of the probe") << endl;
  float probeContactZ;
  if(measureAtSwitch(Z_AXIS, -1, MaxDisplacement + Z_HOME_RETRACT_MM, probeContactZ)) {
    logError
      << F("Unable to measure probe displacement, ")
      << F("the triggering positon of the probe could not be measured (measurement 2 of 2)")
      << endl;
    return -1;
  }

  float displacement = max(0, plateZ - probeContactZ);

  log
    << F("Measuring probe displacement of ") << displacement
    << F(", plateZ=") << plateZ
    << F(", probeContactZ=") << probeContactZ
    << endl;

  if (displacement < MinDisplacement || displacement > MaxDisplacement) {
    logError
      << F("Unable to measure probe displacement, measured value ") << displacement
      << F(" is outside of the expected range")
      << endl;
    return -1;
  }

  o_displacement = displacement;
  return 0;
}
