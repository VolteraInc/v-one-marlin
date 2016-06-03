#include "../api/api.h"
#include "../Marlin.h"

#include "processing.h"

//-------------------------------------------
// Utils/Debugging
int process_dcode(int command_code) {
  switch(command_code) {

    // Enable/disable logging
    case 1:
      // Toggle logging
      logging_enabled = !logging_enabled;
      SERIAL_ECHO_START;
      SERIAL_ECHO(logging_enabled ? "Logging ON\n" : "Logging OFF\n");
      return 0;

    // Algorithms - prepare to move
    case 101:
      return prepareToolToMove();

    // Algorithms - Homing
    case 102: {
      bool home_all = !(code_seen('X') || code_seen('Y') || code_seen('Z'));
      return home(
        home_all || code_seen('X'),
        home_all || code_seen('Y'),
        home_all || code_seen('Z')
      );
    }

    // Algorithms - XY Positioner
    case 103:
      if (prepareToolToMove()) {
        return -1;
      }

      // Check for move-only flag
      if (code_seen('M')) {
        return moveToXyPositioner();

      } else {
        // Find the center
        float centerX;
        float centerY;
        const long cycles = code_seen('C') ? code_value_long() : defaultXyPositionerCycles;
        const int returnValue = xyPositionerFindCenter(cycles, centerX, centerY);

        // Output
        SERIAL_ECHO_START;
        SERIAL_ECHO("xyPositionerFindCenter");
        SERIAL_ECHO(" returnValue:"); SERIAL_ECHO(returnValue);
        SERIAL_ECHO(" cycles:"); SERIAL_ECHO(cycles);
        SERIAL_ECHO(" x:"); SERIAL_ECHO(centerX);
        SERIAL_ECHO(" y:"); SERIAL_ECHO(centerY);
        SERIAL_ECHO("\n");
        return 0;
      }

    // Algorithms - Probe calibration plate
    case 104: {
      if ( prepareToolToMove()
        || moveToXyPositioner()) {
        return -1;
      }

      float displacement = 0.0f;
      const int returnValue = measureProbeDisplacement(displacement);

      // Output
      SERIAL_ECHO_START;
      SERIAL_ECHO("measureProbeDisplacement");
      SERIAL_ECHO(" returnValue:"); SERIAL_ECHO(returnValue);
      SERIAL_ECHO(" displacement:"); SERIAL_ECHO(displacement);
      SERIAL_ECHO("\n");
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
      SERIAL_ECHO("measureAtSwitch");
      SERIAL_ECHO(" returnValue:"); SERIAL_ECHO(returnValue);
      SERIAL_ECHO(" axis:"); SERIAL_ECHO(axis_codes[axis]);
      SERIAL_ECHO(" direction:"); SERIAL_ECHO(direction);
      SERIAL_ECHO(" measurement:"); SERIAL_ECHO(measurement);
      SERIAL_ECHO("\n");
    // Algorithms - Read probe pin voltage
    case 106: {
      const int maxCycles = 50;
      float voltages[maxCycles];
      const int cycles = code_seen('C') ? code_value() : maxCycles;
      const int ms = code_seen('M') ? code_value() : 1;

      if (cycles > maxCycles) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Warning: The requested number of cycles ("); SERIAL_ECHO(cycles);
        SERIAL_ECHOPGM(") exceeds the maximum ("); SERIAL_ECHO(maxCycles);
        SERIAL_ECHOPGM(")");
        return 0;
      }

      // Collect the voltage readings
      for (int i = 0; i < cycles; ++i) {
        voltages[i] = readProbePinVoltage();
        delay(ms);
      }

      // Output the voltage readings
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Voltages");
      for (int i = 0; i < cycles; ++i) {
        SERIAL_ECHOLN(voltages[i]);
      }

      return 0;
    }

    //-------------------------------------------
    // List Commands
    default:
      SERIAL_ECHO_START;
      SERIAL_ECHOLN("Movement Commands");
      SERIAL_ECHOLN("  D1 - Toggle logging ON/OFF (default: OFF)");
      SERIAL_ECHOLN("");
      SERIAL_ECHOLN("Algorithms");
      SERIAL_ECHOLN("  D101 - prepare tool to move");
      SERIAL_ECHOLN("  D102 - Home -- D102 or D102 XY");
      SERIAL_ECHOLN("  D103 - xy positioner -- D103 or D103 M (move-only)");
      SERIAL_ECHOLN("  D104 - probe displacement");
      SERIAL_ECHOLN("  D105 - measure at switch -- D105 -X");
      SERIAL_ECHOLN("  D106 - Read probe voltage -- D106 C50 M1")
      SERIAL_ECHOLN("");
      return 0;
  }
}
