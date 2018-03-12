#include "../../api/api.h"
#include "../../../Marlin.h"
#include "../../../ConfigurationStore.h"
#include "../../vone/tools/Probe.h"

int calibrateSwitchPositions(tools::Probe& probe, unsigned cycles, bool storeResults) {
  if (probe.prepareToMove(tools::PrepareToMove::Options::skipCalibrateXYZ)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Unable to calibrate positions, could not prepare probe");
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

  // Set the x,y position of the z-switch using hardcoded offset values.
  min_z_x_pos = centerX + OFFSET_FROM_XYPOS_TO_MINZ_X;
  min_z_y_pos = centerY + OFFSET_FROM_XYPOS_TO_MINZ_Y;

  // Home Z (uses the new z-switch location)
  if (homeZ(probe)) {
    return -1;
  }

  // Find the center, using the standard algorithm
  if (xyPositionerFindCenter(probe, cycles, centerX, centerY, useConfiguredZ)) {
    return -1;
  }

  // Set the x,y position of the xy-positioner
  xypos_x_pos = centerX;
  xypos_y_pos = centerY;

  // Store positions
  if (storeResults) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Storing settings...");
    Config_StoreCalibration();
  }

  // Success
  return 0;
}
