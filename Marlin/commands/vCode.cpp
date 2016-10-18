#include "../api/api.h"
#include "../Marlin.h"
#include "../Stepper.h"

#include "processing.h"

int process_vcode(int command_code) {
  const auto tool = getTool();
  switch(command_code) {

    //-------------------------------------------
    // Movement Status
    case 0:
      return outputMovementStatus();

    // Move
    case 1:
      if (code_seen('D') && !code_seen('Z')) {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM("Unable to perform movement, D option can not be applied unless Z is given");
        return -1;
      }

      return prepareToolToMove(tool) ||
        asyncMove(
          tool,
          code_seen('X') ? code_value() : current_position[ X_AXIS ],
          code_seen('Y') ? code_value() : current_position[ Y_AXIS ],
          code_seen('Z') ? code_value() : current_position[ Z_AXIS ],
          current_position[ E_AXIS ] + (code_seen('E') ?  code_value() : 0.0f), // E values are always relative
          code_seen('F') ? code_value() : getDefaultFeedrate(),
          code_seen('D') // apply/ignore dispense height
        );

    // Relative move
    case 2:
      return prepareToolToMove(tool) ||
        asyncRelativeMove(
          tool,
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
      if (prepareToolToMove(tool)) {
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
      if (tool != TOOLS_PROBE) {
        SERIAL_ERROR_START;
        SERIAL_ERRORPGM("Unable to probe, current tool is "); SERIAL_ERRORLN(toolTypeAsString(tool));
        return -1;
      }

      float additionalRetractDistance = code_seen('R') ? code_value() : DefaultRetract;
      float measurement;
      if (
        prepareToolToMove(tool) ||
        probe(tool, measurement, additionalRetractDistance)
      ) {
        return -1;
      }

      // Output position
      SERIAL_PROTOCOLPGM("probeMeasurement");
      SERIAL_PROTOCOLPGM(" x:"); SERIAL_PROTOCOL_F(current_position[X_AXIS], 3);
      SERIAL_PROTOCOLPGM(" y:"); SERIAL_PROTOCOL_F(current_position[Y_AXIS], 3);
      SERIAL_PROTOCOLPGM(" z:"); SERIAL_PROTOCOL_F(measurement, 3);
      SERIAL_PROTOCOLPGM("\n");
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

    // Set dispense height
    // Note: Change will be applied to next movement
    case 102:
      return setDispenseHeight(tool, code_seen('Z') ? code_value() : 0.0f);

    //-------------------------------------------
    // Deprecated
    // For compatibility - these V-Commands are now D-Commands but production tests use V110# cmds.
    case 1101: return process_dcode(101);
    case 1103: return process_dcode(103);

    //-------------------------------------------
    // List Commands
    default:
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Movement Commands");
      SERIAL_ECHOLNPGM("  V0 - Movement status");
      SERIAL_ECHOLNPGM("  V1 - Move/Dispense -- V1 X100 Y100 Z10 E30 F6000");
      SERIAL_ECHOLNPGM("  V2 - Relative Move/Dispense -- V2 X5 Y3 Z-1 E2 F6000");
      SERIAL_ECHOLNPGM("  V3 - Move until limit switch triggers -- V3 -X -Y -Z F6000");
      SERIAL_ECHOLNPGM("  V4 - Probe at current position (retract by probe displacement + R) -- V4 R1");
      SERIAL_ECHOLNPGM("  V5 - raise, home XY, and reset tool preparations -- V5");
      SERIAL_ECHOLNPGM("  V6 - override planner's current position -- V6 E0");
      SERIAL_ECHOLNPGM("  V7 - override planner's current position, relatively -- V7 E-1");
      SERIAL_ECHOLNPGM("");
      SERIAL_ECHOLNPGM("Tool Commands");
      SERIAL_ECHOLNPGM("  V100 - Tool status");
      SERIAL_ECHOLNPGM("  V101 - attach/detach tool -- V101 or V101 P or V101 D");
      SERIAL_ECHOLNPGM("  V102 - set dispense height (must have dispenser attached) -- V102 Z0.140");
      return 0;
  }
}
