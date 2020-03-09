#include "../../api/api.h"
#include "../../../Marlin.h"
#include "../../../ConfigurationStore.h"
#include "../../vone/tools/Probe.h"
#include "../../vone/VOne.h"
#include "../../vone/endstops/ScopedEndstopEnable.h"

static int s_calibrateSwitchPositions(tools::Probe& probe, unsigned cycles, bool storeResults) {
  auto& endstopMonitor = vone->stepper.endstopMonitor;
  auto& toolSwitch = vone->endstops.toolSwitch;
  auto& zSwitch = vone->endstops.zSwitch;

  if (probe.prepareToMove(tools::PrepareToMove::Option::skipCalibrateXYZ)) {
    logError << F("Unable to calibrate positions, could not prepare probe") << endl;
    return -1;
  }

  // Find the center of xy-positioner without using configured Z
  // Note: We don't use the configured Z position because we have not homed Z.
  // We don't home Z because we'd need a reliable hardcoded x,y position for
  // the z-switch, which has proven difficult. Meanwhile, the xy-position is
  // more tolerant of inaccuracies in the hardcoded values.
  float centerX;
  float centerY;
  if (xyPositionerFindCenter(probe, cycles, centerX, centerY, usePlateBackOffForZ)) {
    return -1;
  }
  float backoffPositionZ = current_position[Z_AXIS];

  // Compute the position of the z-swith, then test the value
  // Note: disable the tool switch becuase z-switches with
  //       strong springs will cause it to trigger
  float zSwitchX = centerX + OFFSET_FROM_XYPOS_TO_MINZ_X;
  float zSwitchY = centerY + OFFSET_FROM_XYPOS_TO_MINZ_Y;
  float zSwitchMeasurement;
  ScopedEndstop_DISABLE sed(endstopMonitor, toolSwitch);
  if (
    raiseToEndstop() ||
    moveXY(probe, zSwitchX, zSwitchY) ||
    measureAtSwitch(zSwitch, useDefaultMaxTravel, zSwitchMeasurement)
  ) {
    logError << F("Unable to calibrate positions, could not test computed position of z-switch") << endl;
    return -1;
  }



  // Store computed positions
  if (storeResults) {
    log << F("Storing settings...") << endl;
    min_z_x_pos = zSwitchX;
    min_z_y_pos = zSwitchY;
    xypos_x_pos = centerX;
    xypos_y_pos = centerY;
    xypos_z_pos = backoffPositionZ - zSwitchMeasurement;
    Config_StoreCalibration();
  }

  // Success
  return 0;
}

int calibrateSwitchPositions(tools::Probe& probe, unsigned cycles, bool storeResults) {
  int returnValue = s_calibrateSwitchPositions(probe, cycles, storeResults);

  return (
    // Force a normal prepare to be performed by the next operation
    // Note: rather than leave the tool in a weird state
    probe.resetPreparations() ||

    returnValue
  );
}
