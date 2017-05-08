#include "../api/api.h"
#include "../../Marlin.h"
#include "../../temperature.h"

#include "processing.h"

//-------------------------------------------
// Utils/Debugging
int process_dcode(int command_code) {
  const auto tool = getTool();
  switch(command_code) {

    // Enable/disable logging
    case 1:
      // Toggle logging
      logging_enabled = !logging_enabled;
      SERIAL_ECHO_START;
      SERIAL_ECHO(logging_enabled ? F("Logging ON\n") : F("Logging OFF\n"));
      return 0;

    // Algorithms - prepare to move
    case 101:
      return prepareToolToMove(tool);

    // Algorithms - Homing
    case 102: {
      bool home_all = !(code_seen('X') || code_seen('Y') || code_seen('Z'));
      if (code_seen('Z')) {
        if (raise()) {
          return -1;
        }
      }
      return rawHome(
        tool,
        home_all || code_seen('X'),
        home_all || code_seen('Y'),
        home_all || code_seen('Z')
      );
    }

    // Algorithms - XY Positioner
    case 103:
      if (prepareToolToMove(tool)) {
        return -1;
      }

      // Check for move-only flag
      if (code_seen('M')) {
        return moveToXyPositioner(tool);

      } else {
        // Find the center
        float centerX;
        float centerY;
        const long cycles = code_seen('C') ? code_value_long() : defaultXyPositionerCycles;
        const int returnValue = xyPositionerFindCenter(tool, cycles, centerX, centerY);

        // Output
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("xyPositionerFindCenter");
        SERIAL_ECHOPGM(" returnValue:"); SERIAL_ECHO(returnValue);
        SERIAL_ECHOPGM(" cycles:"); SERIAL_ECHO(cycles);
        SERIAL_ECHOPGM(" x:"); SERIAL_ECHO(centerX);
        SERIAL_ECHOPGM(" y:"); SERIAL_ECHO(centerY);
        SERIAL_ECHOPGM("\n");
        return 0;
      }

    // Algorithms - Measure probe displacement
    case 104: {
      if (prepareToolToMove(tool)) {
        return -1;
      }

      float displacement = 0.0f;
      const int returnValue = measureProbeDisplacement(tool, displacement);

      // Output
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("measureProbeDisplacement");
      SERIAL_ECHOPGM(" returnValue:"); SERIAL_ECHO(returnValue);
      SERIAL_ECHOPGM(" displacement:"); SERIAL_ECHO(displacement);
      SERIAL_ECHOPGM("\n");
      return 0;
    }

    // Algorithms - Measure at switch
    case 105: {
      float measurement;
      const int axis = (
        code_seen('X') ? X_AXIS : (
          code_seen('Y') ? Y_AXIS : (
            code_seen('Z') ? Z_AXIS : Z_AXIS
          )
        )
      );
      const int direction = code_prefix() == '-' ? -1 : 1;
      const int returnValue = measureAtSwitch(axis, direction, useDefaultMaxTravel, measurement);

      // Output
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("measureAtSwitch");
      SERIAL_ECHOPGM(" returnValue:"); SERIAL_ECHO(returnValue);
      SERIAL_ECHOPGM(" axis:"); SERIAL_ECHO(axis_codes[axis]);
      SERIAL_ECHOPGM(" direction:"); SERIAL_ECHO(direction);
      SERIAL_ECHOPGM(" measurement:"); SERIAL_ECHOLN(measurement);
      return 0;
    }

    // Algorithms - Read voltage of left pogo pin (aka p_top)
    case 106: {
      const int maxCycles = 250;
      float rawVoltages[maxCycles];
      const int cycles = code_seen('C') ? code_value() : maxCycles;
      const int ms = code_seen('M') ? code_value() : 1;

      if (cycles > maxCycles) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Warning: The requested number of cycles ("); SERIAL_ECHO(cycles);
        SERIAL_ECHOPGM(") exceeds the maximum ("); SERIAL_ECHO(maxCycles);
        SERIAL_ECHOLNPGM(")");
        return 0;
      }

      // Collect the voltage readings
      // Note: set p_top mode to disable ADC reads for a little while
      // (2 seconds). This is hacky but ADC reads use the same interrupt timer
      // as millis() so this is the only way to disable reads without disabling
      // millis()
      set_p_top_mode(P_TOP_COMMS_READ_MODE);
      for (int i = 0; i < cycles; ++i) {
        rawVoltages[i] = analogRead(P_TOP_STATE_PIN);
        delay(ms);
      }
      set_p_top_mode(P_TOP_NORMAL_MODE);

      // Output the voltage readings
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Voltages");
      for (int i = 0; i < cycles; ++i) {
        SERIAL_ECHOLN(rawToVoltage(rawVoltages[i]));
      }

      return 0;
    }

    // Algorithms - Bed height
    case 107: {
      float x = code_seen('X') ? code_value() : current_position[ X_AXIS ];
      float y = code_seen('Y') ? code_value() : current_position[ Y_AXIS ];
      float height = bedHeightAt(x, y);
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("Bed height at ("); SERIAL_ECHO(x);
      SERIAL_ECHOPGM(", "); SERIAL_ECHO(y);
      SERIAL_ECHOPGM(") is "); SERIAL_ECHOLN(height);
      return 0;
    }

    //-------------------------------------------
    // List Commands
    default:
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("General Commands");
      SERIAL_ECHOLNPGM("  D1 - Toggle logging ON/OFF (default: OFF)");
      SERIAL_ECHOLNPGM("");
      SERIAL_ECHOLNPGM("Algorithms");
      SERIAL_ECHOLNPGM("  D101 - prepare tool to move");
      SERIAL_ECHOLNPGM("  D102 - Home -- D102 or D102 XY");
      SERIAL_ECHOLNPGM("  D103 - xy positioner -- D103 or D103 M (move-only)");
      SERIAL_ECHOLNPGM("  D104 - measure probe displacement");
      SERIAL_ECHOLNPGM("  D105 - measure at switch -- D105 -X");
      SERIAL_ECHOLNPGM("  D106 - read left pogo pin's voltage (C=cycles M=milliseconds between readings) -- D106 C10 M5 ");
      SERIAL_ECHOLNPGM("");
      return 0;
  }
}
