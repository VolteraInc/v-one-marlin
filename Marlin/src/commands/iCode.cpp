#include "../../Marlin.h"
#include "../api/api.h"
#include "../api/diagnostics/diagnostics.h"

#include "processing.h"

#include "../../ConfigurationStore.h"
#include "../vone/vone.h"

//-------------------------------------------
// Special commands for iNternal use
int process_icode(int command_code) {
  switch(command_code) {

    // Reset positions to defaults
    case 1:
      xypos_x_pos = XYPOS_X_POS;
      xypos_y_pos = XYPOS_Y_POS;
      min_z_x_pos = MIN_Z_X_POS;
      min_z_y_pos = MIN_Z_Y_POS;

      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Reset Positions to");
      SERIAL_ECHOPAIR(" xyPositioner_x:", xypos_x_pos);
      SERIAL_ECHOPAIR(" xyPositioner_y:", xypos_y_pos);
      SERIAL_ECHOPAIR(" zSwitch_x:", min_z_x_pos);
      SERIAL_ECHOPAIR(" zSwitch_y:", min_z_y_pos);
      SERIAL_EOL;
      return 0;

    // Calibrate the locations of the z-switch and xy-positioner
    case 2: {
      const unsigned cycles = code_seen('C') ? code_value() : defaultSwitchPositionCalibrationCycles;
      if (runCalibrateSwitchPositions(vone->toolBox.probe, cycles)) {
        return -1;
      }

      // Output
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("positionCalibration");
      SERIAL_ECHOPAIR(" cycles:", cycles);
      SERIAL_ECHOPAIR(" xyPositioner_x:", xypos_x_pos);
      SERIAL_ECHOPAIR(" xyPositioner_y:", xypos_y_pos);
      SERIAL_ECHOPAIR(" zSwitch_x:", min_z_x_pos);
      SERIAL_ECHOPAIR(" zSwitch_y:", min_z_y_pos);
      SERIAL_EOL;
      Config_PrintCalibration();

      return 0;
    }

    // Run burn-in sequence
    case 3:
      return runBurnInSequence(vone->toolBox.nullTool);

    //-------------------------------------------
    // Error if command unknown
    default:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unknown command");
      return -1;
  }
}
