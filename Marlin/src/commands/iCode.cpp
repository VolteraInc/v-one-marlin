#include "../api/api.h"
#include "../../Marlin.h"

#include "processing.h"

#include "../../ConfigurationStore.h"

//-------------------------------------------
// Special commands for iNternal use
int process_icode(int command_code) {
  const auto tool = getTool();
  switch(command_code) {

    // Reset positions to defaults
    case 1:
      xypos_x_pos = XYPOS_X_POS;
      xypos_y_pos = XYPOS_Y_POS;
      min_z_x_pos = MIN_Z_X_POS;
      min_z_y_pos = MIN_Z_Y_POS;

      SERIAL_ECHOPGM("Reset Positions to\n");
      SERIAL_ECHOPGM(" xyPositioner_x:"); SERIAL_ECHOLN(xypos_x_pos);
      SERIAL_ECHOPGM(" xyPositioner_y:"); SERIAL_ECHOLN(xypos_y_pos);
      SERIAL_ECHOPGM(" zSwitch_x:"); SERIAL_ECHOLN(min_z_x_pos);
      SERIAL_ECHOPGM(" zSwitch_y:"); SERIAL_ECHOLN(min_z_y_pos);
      return 0;

    // Calibrate the locations of the z-switch and xy-positioner
    case 2: {
      const long defaultCycles = 5;
      const long cycles = code_seen('C') ? code_value_long() : defaultCycles;
      if (calibrateKeyPositions(tool, cycles)) {
        return -1;
      }

      // Output
      SERIAL_PROTOCOLPGM("positionCalibration");
      SERIAL_PROTOCOLPGM(" cycles:"); SERIAL_PROTOCOL(cycles);
      SERIAL_PROTOCOLPGM(" xyPositioner_x:"); SERIAL_PROTOCOL(xypos_x_pos);
      SERIAL_PROTOCOLPGM(" xyPositioner_y:"); SERIAL_PROTOCOL(xypos_y_pos);
      SERIAL_PROTOCOLPGM(" zSwitch_x:"); SERIAL_PROTOCOL(min_z_x_pos);
      SERIAL_PROTOCOLPGM(" zSwitch_y:"); SERIAL_PROTOCOL(min_z_y_pos);
      SERIAL_PROTOCOLPGM("\n");

      SERIAL_PROTOCOLLNPGM("Storing settings...");
      Config_StoreCalibration();
      Config_RetrieveCalibration();

      Config_PrintCalibration();

      return 0;
    }

    case 3: {
      const bool shouldSave = code_seen('S');
      const int gridSize = code_seen('G') ? code_value() : 4;
      if (mapBed(tool, gridSize)) {
        return -1;
      }

      outputBedMap();

      if (shouldSave) {
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("TODO: Storing bed mapping");
      }

      return 0;
    }

    //-------------------------------------------
    // Error if command unknown
    default:
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Unknown command");
      return -1;
  }
}
