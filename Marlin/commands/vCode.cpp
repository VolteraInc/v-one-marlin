#include "../api/api.h"
#include "../Marlin.h"

#include "processing.h"

int process_vcode(int command_code) {
  switch(command_code) {

    //-------------------------------------------
    // Movement Status
    case 0:
      return outputMovementStatus();

    // Move
    case 1:
      return prepareToolToMove() ||
        move(
          code_seen('X') ? code_value() : current_position[ X_AXIS ],
          code_seen('Y') ? code_value() : current_position[ Y_AXIS ],
          code_seen('Z') ? code_value() : current_position[ Z_AXIS ],
          code_seen('E') ? code_value() : current_position[ E_AXIS ],
          code_seen('F') ? code_value() : getDefaultFeedrate()
        );

    // Relative move
    case 2:
      return prepareToolToMove() ||
        relativeMove(
          code_seen('X') ? code_value() : 0,
          code_seen('Y') ? code_value() : 0,
          code_seen('Z') ? code_value() : 0,
          code_seen('E') ? code_value() : 0,
          code_seen('F') ? code_value() : getDefaultFeedrate()
        );

    // Move until limit switch triggers
    // Notes:
    //    - movement is performed one axis at a time.
    //    - to avoid crashes we always raise first (if needed) and lower last (if needed).
    case 3: {
      if (prepareToolToMove()) {
        return -1;
      }
      const float feedrate = code_seen('F') ? code_value() : getDefaultFeedrate();
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

    // Probe at current x,y position
    case 4: {
      if (getTool() != TOOLS_PROBE) {
        SERIAL_ERROR_START;
        SERIAL_ERROR("Unable to probe, current tool is ");
        SERIAL_ERRORLN(toolTypeAsString(getTool()));
        return -1;
      }

      float measurement;
      if (
        prepareToolToMove() ||
        probe(measurement)
      ) {
        return -1;
      }

      // Output position
      SERIAL_PROTOCOL("probeMeasurement");
      SERIAL_PROTOCOL(" x:"); SERIAL_PROTOCOL_F(current_position[X_AXIS], 3);
      SERIAL_PROTOCOL(" y:"); SERIAL_PROTOCOL_F(current_position[Y_AXIS], 3);
      SERIAL_PROTOCOL(" z:"); SERIAL_PROTOCOL_F(measurement, 3);
      SERIAL_PROTOCOL("\n");
      return 0;
    }

    // Home/Force re-prep
    case 5:
      setHomedState(X_AXIS, 0);
      setHomedState(Y_AXIS, 0);
      setHomedState(Z_AXIS, 0);
      return (
        resetToolPreparations() ||
        homeXY()
      );


    //-------------------------------------------
    // Tool status
    case 100:
      return outputToolStatus();

    // Attach/Detach tool
    case 101:
      if (code_seen('P')) setTool(TOOLS_PROBE);
      else if (code_seen('D')) setTool(TOOLS_DISPENSER);
      else setTool(TOOLS_NONE);
      return 0;

    //-------------------------------------------
    // Utils/Debugging

    // Enable/disable logging
    case 1001:
      if (code_seen('D')) {
        // Turn on debug logging
        logging_enabled = true;
      } else {
        // Return to default log level
        SERIAL_ECHO_START;
        SERIAL_ECHO("Restoring default log level\n");
        logging_enabled = false;
      }

      // Output current log level
      SERIAL_ECHO_START;
      SERIAL_ECHO("LoggingLevel:");
      SERIAL_ECHO(logging_enabled ? "Debug" : "Warning");
      SERIAL_ECHO("\n");
      return 0;

    // Algorithms - prepare to move
    case 1101:
      return prepareToolToMove();

    // Algorithms - Homing
    case 1102: {
      bool home_all = !(code_seen('X') || code_seen('Y') || code_seen('Z'));
      return home(
        home_all || code_seen('X'),
        home_all || code_seen('Y'),
        home_all || code_seen('Z')
      );
    }

    // Algorithms - XY Positioner
    case 1103:
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
    case 1104: {
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
    case 1105: {
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
      return 0;
    }

    //-------------------------------------------
    // List Commands
    default:
      SERIAL_ECHO_START;
      SERIAL_ECHOLN("Movement Commands");
      SERIAL_ECHOLN("  V0 - Movement status");
      SERIAL_ECHOLN("  V1 - Move/Dispense -- V1 X100 Y100 Z10 E30 F6000");
      SERIAL_ECHOLN("  V2 - Relative Move/Dispense -- V2 X5 Y3 Z-1 E2 F6000");
      SERIAL_ECHOLN("  V3 - Move until limit switch triggers -- V3 -X -Y -Z F6000");
      SERIAL_ECHOLN("  V4 - Probe at current position -- V4");
      SERIAL_ECHOLN("  V5 - raise, home XY, and reset tool preparations -- V5");
      SERIAL_ECHOLN("");
      SERIAL_ECHOLN("Tool Commands");
      SERIAL_ECHOLN("  V100 - Tool status");
      SERIAL_ECHOLN("  V101 - attach/detach tool -- V101 or V101 P or V101 D");
      SERIAL_ECHOLN("");
      return 0;
  }
}
