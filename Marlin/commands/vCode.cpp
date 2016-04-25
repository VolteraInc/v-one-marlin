#include "../api/api.h"
#include "../Marlin.h"

#include "processing.h"

//TODO: set to highest axis speed
const float default_feedrate = 15000.0f;

int process_vcode(int command_code) {
  switch(command_code) {

    //-------------------------------------------
    // Movement Status
    case 0:
      return outputMovementStatus();

    // Move
    case 1:
      return move(
        code_seen('X') ? code_value() : current_position[ X_AXIS ],
        code_seen('Y') ? code_value() : current_position[ Y_AXIS ],
        code_seen('Z') ? code_value() : current_position[ Z_AXIS ],
        code_seen('E') ? code_value() : current_position[ E_AXIS ],
        code_seen('F') ? code_value() : default_feedrate
      );

    // Relative move
    case 2:
      return relativeMove(
        code_seen('X') ? code_value() : 0,
        code_seen('Y') ? code_value() : 0,
        code_seen('Z') ? code_value() : 0,
        code_seen('E') ? code_value() : 0,
        code_seen('F') ? code_value() : default_feedrate
      );

    // Move until limit switch triggers
    // Notes:
    //    - movement is performed one axis at a time.
    //    - to avoid crashes we always raise first (if needed) and lower last (if needed).
    case 3: {
      const float feedrate = code_seen('F') ? code_value() : default_feedrate;
      if (code_seen('Z') && code_prefix() == '-') {
        if (moveToLimit(Z_AXIS, -1, feedrate)) { return -1; }
      }
      if (code_seen('X')) {
        const auto direction = code_prefix() == '-' ? -1 : 1;
        if (moveToLimit(X_AXIS, direction, feedrate)) { return -1; }
      }
      if (code_seen('Y')) {
        const auto direction = code_prefix() == '-' ? -1 : 1;
        if (moveToLimit(Y_AXIS, direction, feedrate)) { return -1; }
      }
      if (code_seen('Z') && code_prefix() != '-') {
        if (moveToLimit(Z_AXIS, 1, feedrate)) { return -1; }
      }
      return 0;
    }

    //-------------------------------------------
    // Temperature Status
    // V100 - Temperature Status
    // V101 - Set target temperature
    case 100:
      return 0;

    //-------------------------------------------
    // Tools
    // V200 - Tool status
    // V201 - Attach/Detach tool
    // V202 - Calibrate current tool, set cal values or clear calibration ('V102' no args)
    case 200:
      return 0;

    //-------------------------------------------
    // List Commands
    default:
      SERIAL_ECHO_START;
      SERIAL_ECHO("Movement Commands\n");
      SERIAL_ECHO("  V0 - Status\n");
      SERIAL_ECHO("  V1 - Move/Dispense -- V1 X100 Y100 Z10 E30 F6000\n");
      SERIAL_ECHO("  V2 - Relative Move/Dispense -- V2 X5 Y3 Z-1 E2 F6000\n");
      SERIAL_ECHO("  V3 - Move until limit switch triggers -- V3 -X -Y -Z F6000\n");
      SERIAL_ECHO("\n");
      SERIAL_ECHO("Temperature Commands");
      SERIAL_ECHO("  V100 - Status");
      SERIAL_ECHO("\n");
      SERIAL_ECHO("Tool Commands");
      SERIAL_ECHO("  V200 - Status");
      SERIAL_ECHO("\n");
      return 0;
  }
}
