#include "../../Marlin.h"
#include "../api/api.h"
#include "../api/diagnostics/diagnostics.h"

#include "processing.h"

#include "../../ConfigurationStore.h"
#include "../vone/VOne.h"

//-------------------------------------------
// Special commands for internal use
int process_icode(int command_code) {
  switch(command_code) {

    // Reset positions to defaults
    case 1:
      xypos_x_pos = XYPOS_X_POS;
      xypos_y_pos = XYPOS_Y_POS;
      xypos_z_pos = XYPOS_Z_POS;
      min_z_x_pos = MIN_Z_X_POS;
      min_z_y_pos = MIN_Z_Y_POS;

      log
        << F("Reset Positions to")
        << F(" xyPositioner_x:") << xypos_x_pos
        << F(" xyPositioner_y:") << xypos_y_pos
        << F(" xyPositioner_Z:") << xypos_z_pos
        << F(" zSwitch_x:") << min_z_x_pos
        << F(" zSwitch_y:") << min_z_y_pos
        << endl;
      return 0;

    // Calibrate the locations of the z-switch and xy-positioner
    case 2: {
      const unsigned cycles = code_seen('C') ? code_value() : defaultSwitchPositionCalibrationCycles;
      if (runCalibrateSwitches(vone->toolBox.probe, cycles)) {
        return -1;
      }

      // Output
      log
        << F("positionCalibration")
        << F(" cycles:") << cycles
        << F(" xyPositioner_x:") << xypos_x_pos
        << F(" xyPositioner_y:") << xypos_y_pos
        << F(" xyPositioner_z:") << xypos_z_pos
        << F(" zSwitch_x:") << min_z_x_pos
        << F(" zSwitch_y:") << min_z_y_pos
        << endl;
      Config_PrintCalibration();

      return 0;
    }

    // Run burn-in sequence
    case 3:
      return runBurnInSequence(vone->toolBox.nullTool);

    //-------------------------------------------
    // Error if command unknown
    default:
      logError << F("Unknown command") << endl;
      return -1;
  }
}
