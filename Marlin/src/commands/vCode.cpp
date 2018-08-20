#include "processing.h"

#include "../api/api.h"
#include "../../Marlin.h"
#include "../work/work.h"

#include "../vone/VOne.h"

int prepareTool(tools::Tool& tool, bool xyzMoves, bool eMoves) {
  // Note: Doing a full prepare for an E-only move would be unexpected
  if (eMoves && !xyzMoves) {
    return tool.prepareToMove(tools::PrepareToMove::Option::eOnly);
  } else {
    return tool.prepareToMove();
  }
}

int process_vcode(int command_code) {
  auto& currentTool = vone->toolBox.currentTool();
  switch(command_code) {

    //-------------------------------------------
    // Movement Status
    case 0:
      return outputMovementStatus();

    // Move
    case 1: {
      // Prepare, if necessary
      // Note: preparing for an E-only move would be unexpected
      const auto xyzMoves = code_seen('X') || code_seen('Y') || code_seen('Z');
      if (prepareTool(currentTool, xyzMoves, code_seen('E'))) {
        return -1;
      }

      const float x = code_seen('X') ? code_value() : current_position[ X_AXIS ];
      const float y = code_seen('Y') ? code_value() : current_position[ Y_AXIS ];
      const float z = code_seen('Z') ? code_value() : current_position[ Z_AXIS ];
      const float e = current_position[ E_AXIS ] + (code_seen('E') ?  code_value() : 0.0f); // E values are always relative
      const float f = code_seen('F') ? code_value() : getDefaultFeedrate();

      if (code_seen('D')) {
        if (!code_seen('Z')) {
          logError << F("Unable to perform movement, D option can not be applied unless Z is given") << endl;
          return -1;
        }

        auto& dispenser = vone->toolBox.dispenser;
        if (confirmAttached("apply D option to movement command", dispenser)) {
          return -1;
        }

        return dispenser.enqueueDispense(x, y, z, e, f);
      } else {
        return currentTool.enqueueMove(x, y, z, e, f);
      }
    }

    // Relative move
    case 2: {
      // Prepare, if necessary
      const auto xyzMoves = code_seen('X') || code_seen('Y') || code_seen('Z');
      if (prepareTool(currentTool, xyzMoves, code_seen('E'))) {
        return -1;
      }

      return asyncRelativeMove(
        currentTool,
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
      if (currentTool.prepareToMove()) {
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
      auto& probe = vone->toolBox.probe;
      if (probe.detached()) {
        logError << F("Unable to probe, current tool is ") << currentTool.name() << endl;
        return -1;
      }

      const auto additionalRetractDistance = code_seen('R') ? code_value() : tools::Probe::DefaultRetract;
      const auto speed = code_seen('F') ? code_value() : tools::Probe::DefaultSpeed;
      const auto maxSamples = code_seen('S') ? code_value() : tools::Probe::DefaultMaxSamples;
      const auto maxTouchesPerSample = code_seen('T') ? code_value() : tools::Probe::DefaultMaxTouchesPerSample;
      auto samplesTaken = 0u;
      auto touchesUsed = 0u;
      auto measurement = -9999.9f;
      if (
        probe.prepareToMove() ||
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
      protocol
        << F("probeMeasurement")
        << F(" x:") << current_position[X_AXIS]
        << F(" y:") << current_position[Y_AXIS]
        << F(" z:") << measurement
        << F(" displacement:") << probe.displacement()
        << F(" samplesTaken:") << samplesTaken
        << F(" touchesUsed:") << touchesUsed
        << endl;
      return 0;
    }

    // Home/Force re-prep
    case 5:
      setHomedState(X_AXIS, 0);
      setHomedState(Y_AXIS, 0);
      setHomedState(Z_AXIS, 0);
      if (currentTool.resetPreparations()) {
        return -1;
      }
      vone->stepper.resume();
      return homeXY(currentTool);

    //-------------------------------------------
    // Tool status
    case 100: {
      const auto& tb = vone->toolBox;
      log << F("Tool") << endl;
      log << F("  type: ") << currentTool.name() << endl;
      log << F("  prepared: ") << currentTool.prepared() << endl;

      tb.probe.outputStatus();

      log << F("Dispenser") << endl;
      log << F("  dispense height: ") << tb.dispenser.dispenseHeight() << endl;

      log << F("Drill") << endl;
      log << F("  Speed: ") << tb.drill.rotationSpeed() << endl;

      log << F("Homing") << endl;
      log << F("  x: ") << getHomedState(X_AXIS) << endl;
      log << F("  y: ") << getHomedState(Y_AXIS) << endl;
      log << F("  z: ") << getHomedState(Z_AXIS) << endl;

      return 0;
    }

    // Attach/Detach tool
    case 101: {
      // Disable tool detection if Force option included
      vone->toolDetector.enable(!code_seen('F'));

      // Set Tool
      if      (code_seen('P')) vone->toolBox.setTool(&vone->toolBox.probe);
      else if (code_seen('D')) vone->toolBox.setTool(&vone->toolBox.dispenser);
      else if (code_seen('R')) vone->toolBox.setTool(&vone->toolBox.drill);
      else                     vone->toolBox.setTool(&vone->toolBox.nullTool);

      return 0;
    }

    // Set dispense height
    // Note: Change will be applied to next movement
    case 102: {
      auto& dispenser = vone->toolBox.dispenser;
      return (
        confirmAttached("set dispensing height", dispenser) ||
        dispenser.setDispenseHeight(code_seen('Z') ? code_value() : 0.0f)
      );
    }

    // Set rotation speed
    case 110: {
      auto& drill = vone->toolBox.drill;
      return (
        confirmAttached("set rotation speed", drill) ||
        drill.prepareToMove() ||
        drill.setRotationSpeed(code_seen('R') ? code_value() : 0.0f)
      );
    }

    // Probe height safety
    // Note: if we ever need to set the safe height
    //       we can use "V201 Z1.5"
    case 201: {
      auto& probe = vone->toolBox.probe;
      if  (code_seen('E')) {
        const auto enable = code_value() == 1;
        probe.enableHeightSafety(enable);
      }
      probe.outputStatus();
      return 0;
    }

    // Probe hole
    case 211: {
      using namespace probing;
      if (!code_seen('D')) {
        logError << F("Unable to probe hole, no diameter given") << endl;
        return -1;
      }
      const auto holeDiameter = code_value();
      const Point2d center = {
        code_seen('X') ? code_value() : current_position[X_AXIS],
        code_seen('Y') ? code_value() : current_position[Y_AXIS]
      };
      const auto MaxMeasurements = 100;
      Point3d measurements[MaxMeasurements];
      auto numMeasurements = 0u;
      auto& probe = vone->toolBox.probe;
      if (
        confirmAttached("probe hole", probe) ||
        probe.prepareToMove() ||
        probeHole(
          probe,
          center,
          holeDiameter / 2,
          measurements,
          MaxMeasurements,
          &numMeasurements
        )
      ) {
        return -1;
      }

      protocol << F("{ \"probeHoleMeasurement\": { \"measurements\": [");
      for (auto idx = 0u; idx < numMeasurements; ++idx) {
        protocol
          << (idx != 0 ? F(", ") : F(""))
          << F("{ \"x\":") << measurements[idx].x
          << F(", \"y\":") << measurements[idx].y
          << F(", \"z\":") << measurements[idx].z
          << F(" }");
      }
      protocol
        << F("] } }")
        << endl;
      return 0;
    }

    //-------------------------------------------
    // Deprecated
    // For compatibility - these V-Commands are now D-Commands but production tests use V110# cmds.
    case 1101: return process_dcode(101);
    case 1103: return process_dcode(103);

    //-------------------------------------------
    // List Commands
    default:
      log << F("V-Commands") << endl;
      log << F("  smart commands to simplify interactions. These commands will") << endl;
      log << F("  automatically home, calibrate tool, etc. if needed ") << endl;
      log << endl;

      log << F("Movement Commands") << endl;
      log << F("  V0 - Movement status") << endl;
      log << F("  V1 - Move/Dispense -- V1 X100 Y100 Z10 E30 F6000") << endl;
      log << F("  V2 - Relative Move/Dispense -- V2 X5 Y3 Z-1 E2 F6000") << endl;
      log << F("  V3 - Move until limit switch triggers -- V3 -X -Y -Z F6000") << endl;
      log << F("  V5 - raise, home XY, and reset tool preparations -- V5") << endl;
      log << endl;

      log << F("Tool Commands") << endl;
      log << F("  General") << endl;
      log << F("    V100 - Tool status") << endl;
      log << F("    V101 - detach tool (i.e. set tool to none), attach options listed below, include 'F' to force change") << endl;
      log << F("  Dispenser") << endl;
      log << F("    V101 D - attach dispenser, include 'F' to force change") << endl;
      log << F("    V102 - set dispense height (must have dispenser attached) -- V102 Z0.140") << endl;
      log << F("  Probe") << endl;
      log << F("    V101 P - attach probe, include 'F' to force change") << endl;
      log << F("    V4   - Probe point at current position (retract by probe displacement + R) -- V4 R1") << endl;
      log << F("    V201 - E1 to enable, E0 to disable height safety -- V201 E1") << endl;
      log << F("    V211 - Probe hole with diameter D centered at X,Y (default to current position) -- V211 D1.0") << endl;
      log << F("  Drill") << endl;
      log << F("    V101 R - attach drill, include 'F' to force change") << endl;
      log << F("    V110 - set drill rotation speed -- V110 R100, no value or 0 means stop") << endl;
      return 0;
  }
}
