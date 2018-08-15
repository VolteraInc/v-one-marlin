#include "../../api/api.h"
#include "../../../Marlin.h"
#include "../../../ConfigurationStore.h"
#include "../../vone/VOne.h"
#include "../../vone/tools/Probe.h"

// Confirm the minY and xyMinY switches are far enough apart.
// If these switches are too close it's possible we'll hit the
// minY switch instead of the xyMinY switch
int checkBackSwitchSeparation(tools::Tool& tool) {
  float measurements[3];
  static const float maxTravel = 10;
  const auto& xyPositionerBack = vone->endstops.xyPositionerBack;
  if (
    raise() ||
    moveToXyPositioner(tool) ||
    measureAtSwitch(xyPositionerBack, maxTravel, measurements[0]) ||
    measureAtSwitch(xyPositionerBack, maxTravel, measurements[1]) ||
    measureAtSwitch(xyPositionerBack, maxTravel, measurements[2])
  ) {
    return -1;
  }

  log
    << F("measurements = [")
    << ArrayWithSize<float>(measurements, sizeof(measurements))
    << F("]")
    << endl;

  float tolerance = 10.000; // TODO: set to a proper value
  if (
    measurements[0] < tolerance ||
    measurements[1] < tolerance ||
    measurements[2] < tolerance
  ) {
    logError
      << F("Unable to calibration switch positions, distance to from ")
      << F("the back switch (y-min)")
      << F(" to ")
      << F("the xy-positioner's back switch (xy-min-y)")
      << F(" exceeded ")
      << FloatWithFormat(tolerance, 2)
      << F("mm, measurements = [")
      << ArrayWithSize<float>(measurements, sizeof(measurements))
      << F("]")
      << endl;
    return -1;
  }

  // Success
  return 0;
}

int calibrateSwitchPositions(tools::Probe& probe, unsigned cycles, bool storeResults) {
  if (probe.prepareToMove(tools::PrepareToMove::Options::skipCalibrateXYZ)) {
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
    log << F("Storing settings...") << endl;
    Config_StoreCalibration();
  }

  // Success
  return 0;
}
