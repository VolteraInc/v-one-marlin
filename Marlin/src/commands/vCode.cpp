#include "../api/api.h"
#include "../../Marlin.h"
#include "../work/work.h"

#include "processing.h"

int process_vcode(int command_code) {
  const auto& currentTool = vone->toolDetector.currentTool();
  switch(command_code) {

    //-------------------------------------------
    // Movement Status
    case 0:
      return outputMovementStatus();

    // Move
    case 1: {
      // Perpare, if necessary
      // Note: preparing for an E-only move would be unexpected
      auto xyzMoves = code_seen('X') || code_seen('Y') || code_seen('Z');
      if (xyzMoves) {
        if (currentTool.prepareToolToMove()) {
          return -1;
        }
      }

      return currentTool.asyncMove(
        code_seen('X') ? code_value() : current_position[ X_AXIS ],
        code_seen('Y') ? code_value() : current_position[ Y_AXIS ],
        code_seen('Z') ? code_value() : current_position[ Z_AXIS ],
        current_position[ E_AXIS ] + (code_seen('E') ?  code_value() : 0.0f), // E values are always relative
        code_seen('F') ? code_value() : getDefaultFeedrate(),
        code_seen('D') // apply/ignore dispense height
      );
    }

    // Relative move
    case 2: {
      // Perpare, if necessary
      // Note: preparing for an E-only move would be unexpected
      auto xyzMoves = code_seen('X') || code_seen('Y') || code_seen('Z');
      if (xyzMoves) {
        if (currentTool.prepareToolToMove()) {
          return -1;
        }
      }

      return currentTool.asyncRelativeMove(
        code_seen('X') ? code_value() : 0,
        code_seen('Y') ? code_value() : 0,
        code_seen('Z') ? code_value() : 0,
        code_seen('E') ? code_value() : 0,
        code_seen('F') ? code_value() : getDefaultFeedrate()
      );
    }

    // Move until limit switch triggers
    // Notes:
    //    - movement is performed one axis at a time.
    //    - to avoid crashes we always raise first (if needed) and lower last (if needed).
    case 3: {
      if (currentTool.prepareToolToMove()) {
        return -1;
      }
      const auto feedrate = code_seen('F') ? code_value() : getDefaultFeedrate();
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
      const auto& probe = vone->tools.probe;
      if (probe.detached() {
        SERIAL_ERROR_START;
        SERIAL_PAIR("Unable to probe, current tool is ", currentTool.typeName());
        SERIAL_EOL;
        return -1;
      }

      const auto additionalRetractDistance = code_seen('R') ? code_value() : DefaultRetract;
      const auto speed = code_seen('F') ? code_value() : Probe::DefaultSpeed;
      const auto maxSamples = code_seen('S') ? code_value() : Probe::DefaultMaxSamples;
      const auto maxTouchesPerSample = code_seen('T') ? code_value() : Probe::DefaultMaxTouchesPerSample;
      auto samplesTaken = 0u;
      auto touchesUsed = 0u;
      auto measurement = -9999.9f;
      if (
        probe.prepareToolToMove() ||
        probe.probe(
          measurement,
          speed, additionalRetractDistance,
          maxSamples, maxTouchesPerSample,
          &samplesTaken, &touchesUsed
        )
      ) {
        return -1;
      }

      // Output position
      SERIAL_PROTOCOLPGM("probeMeasurement");
      SERIAL_PAIR(" x:", current_position[X_AXIS]);
      SERIAL_PAIR(" y:", current_position[Y_AXIS]);
      SERIAL_PAIR(" z:", measurement);
      SERIAL_PAIR(" displacement:", Probe::getProbeDisplacement());
      SERIAL_PAIR(" samplesTaken:", samplesTaken);
      SERIAL_PAIR(" touchesUsed:", touchesUsed);
      SERIAL_EOL;
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
      vone->toolDetector.outputToolStatus();
      return 0;

    // Attach/Detach tool
    case 101: {
      // Disable tool detection if Force option included
      enableToolDetection(!code_seen('F'));

      // Set Tool
      if      (code_seen('P')) vone->setTool(vone->tools.probe);
      else if (code_seen('D')) vone->setTool(vone->tools.dispenser);
      else if (code_seen('R')) vone->setTool(vone->tools.router);
      else                     vone->setTool(vone->tools.nullTool);

      return 0;
    }

    // Set dispense height
    // Note: Change will be applied to next movement
    case 102: {
      // Confirm we have a dispenser
      const auto& dispenser = vone->tools.dispenser;
      if (dispenser.detached()) {
        SERIAL_ERROR_START;
        SERIAL_PAIR("Unable to set dispensing height, current tool is ", currentTool.typeName());
        SERIAL_EOL
        return -1;
      }
      return dispenser.setDispenseHeight(code_seen('Z') ? code_value() : 0.0f);
    }

    // Set rotation speed
    case 110: {
      const auto& router = vone->tools.router;
      if (router.detached()) {
        SERIAL_ERROR_START;
        SERIAL_PAIR("Unable to set rotation speed, current tool is ", currentTool.typeName());
        SERIAL_EOL
        return -1;
      }
      return (
        router.prepareToolToMove() ||
        router.setRotationSpeed(tool, code_seen('R') ? code_value() : 0.0f)
      );
    }

    //-------------------------------------------
    // Deprecated
    // For compatibility - these V-Commands are now D-Commands but production tests use V110# cmds.
    case 1101: return process_dcode(101);
    case 1103: return process_dcode(103);

    //-------------------------------------------
    // List Commands
    default:
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("V-Commands");
      SERIAL_ECHOLNPGM("  smart commands to simplify interactions. These commands will");
      SERIAL_ECHOLNPGM("  automatically home, calibrate tool, etc. if needed ");
      SERIAL_ECHOLNPGM("Movement Commands");
      SERIAL_ECHOLNPGM("  V0 - Movement status");
      SERIAL_ECHOLNPGM("  V1 - Move/Dispense -- V1 X100 Y100 Z10 E30 F6000");
      SERIAL_ECHOLNPGM("  V2 - Relative Move/Dispense -- V2 X5 Y3 Z-1 E2 F6000");
      SERIAL_ECHOLNPGM("  V3 - Move until limit switch triggers -- V3 -X -Y -Z F6000");
      SERIAL_ECHOLNPGM("  V5 - raise, home XY, and reset tool preparations -- V5");
      SERIAL_ECHOLNPGM("");
      SERIAL_ECHOLNPGM("Tool Commands");
      SERIAL_ECHOLNPGM("  General");
      SERIAL_ECHOLNPGM("    V100 - Tool status");
      SERIAL_ECHOLNPGM("    V101 - detach tool (i.e. set tool to none), attach options listed below, include 'F' to force change");
      SERIAL_ECHOLNPGM("  Dispenser");
      SERIAL_ECHOLNPGM("    V101 D - attach dispenser, include 'F' to force change");
      SERIAL_ECHOLNPGM("    V102 - set dispense height (must have dispenser attached) -- V102 Z0.140");
      SERIAL_ECHOLNPGM("  Probe");
      SERIAL_ECHOLNPGM("    V101 P - attach probe, include 'F' to force change");
      SERIAL_ECHOLNPGM("    V4 - Probe at current position (retract by probe displacement + R) -- V4 R1");
      SERIAL_ECHOLNPGM("  Router");
      SERIAL_ECHOLNPGM("    V101 R - attach router, include 'F' to force change");
      SERIAL_ECHOLNPGM("    V110 - set router rotation speed -- V110 R100, no value or 0 means stop");
      return 0;
  }
}
