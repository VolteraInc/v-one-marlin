#include "../api/api.h"
#include "../Marlin.h"
#include "../Stepper.h"

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

    // Set current position
    case 6:
      if (code_seen('X') || code_seen('Y') || code_seen('Z')) {
        return setPosition(
          code_seen('X') ? code_value() : current_position[ X_AXIS ],
          code_seen('Y') ? code_value() : current_position[ Y_AXIS ],
          code_seen('Z') ? code_value() : current_position[ Z_AXIS ],
          code_seen('E') ? code_value() : current_position[ E_AXIS ]
        );
      } else if (code_seen('E')) {
        return setPositionEOnly(code_value());
      }
      return 0;

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
    // Deprecated
    // For compatibility - these V-Commands are now D-Commands but production tests use V110# cmds.
    case 1101:
      process_dcode(101);
      break;

    case 1103:
      process_dcode(103);
      break;

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
      SERIAL_ECHOLN("  V6 - override planner's current position -- V6 E0");
      SERIAL_ECHOLN("");
      SERIAL_ECHOLN("Tool Commands");
      SERIAL_ECHOLN("  V100 - Tool status");
      SERIAL_ECHOLN("  V101 - attach/detach tool -- V101 or V101 P or V101 D");
      SERIAL_ECHOLN("");
      return 0;
  }
}
