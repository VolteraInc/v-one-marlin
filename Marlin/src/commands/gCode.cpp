#include "processing.h"

#include "../api/api.h"
#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../planner.h"

#include "../api/movement/motion_control.h"

#include "../vone/VOne.h"

static float destination[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
static float offset[3] = {0.0, 0.0, 0.0};
static float feedrate = 1500.0;
static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static void s_get_coordinates()
{
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode) * current_position[i];
    } else {
      destination[i] = current_position[i];
    }
  }

  if(code_seen('F')) {
    auto f = code_value();
    if(f > 0.0) feedrate = f;
  }
}

static void s_clamp_to_software_endstops(float target[3]) {
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

static void s_prepare_move() {
  s_clamp_to_software_endstops(destination);

  if (logging_enabled) {
    log
     << F("s_prepare_move")
     << F(" X:") << destination[ X_AXIS ]
     << F(" Y:") << destination[ Y_AXIS ]
     << F(" Z:") << destination[ Z_AXIS ]
     << F(" E:") << destination[ E_AXIS ]
     << F(" F:") << feedrate
     << endl;
  }

  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0);
  }
  memcpy(current_position, destination, sizeof(current_position));
}

static void s_get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   s_get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

static void s_prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(
    vone->toolBox.currentTool(),
    current_position,
    destination,
    offset,
    X_AXIS, Y_AXIS, Z_AXIS,
    feedrate * feedmultiply / 100.0,
    r,
    isclockwise
  );

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  memcpy(current_position, destination, sizeof(current_position));
}

int process_gcode(int command_code) {
  // look here for descriptions of G-codes: http://linuxcnc.org/handbook/gcode/g-code.html
  // http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

  switch(command_code) {
    // G0, G1  - Coordinated Movement X Y Z E F
    case 0:
    case 1:
      s_get_coordinates(); // For X Y Z E F
      s_prepare_move();
      return 0;

    // G2  - CW ARC
    case 2:
      s_get_arc_coordinates();
      s_prepare_arc_move(true);
      return 0;

    // G3  - CCW ARC
    case 3:
      s_get_arc_coordinates();
      s_prepare_arc_move(false);
      return 0;

    // G4  - Dwell S<seconds> or P<milliseconds>
    case 4: {
      unsigned long waitUntil = 0;
      if(code_seen('P')) waitUntil = code_value(); // milliseconds to wait
      if(code_seen('S')) waitUntil = code_value() * 1000; // seconds to wait

      st_synchronize();
      waitUntil += millis();  // keep track of when we started waiting
      while(millis() < waitUntil) {
        periodic_work();
      }
      return 0;
    }

    // G18: XYPositioner Y1 - Move in +Y until a switch is triggered
    case 18: {
      float measurement;
      if ( xyPositionerTouch(vone->endstops.xyPositionerForward, measurement)
        || moveXY(vone->toolBox.probe, xypos_x_pos, xypos_y_pos)) {
        return -1;
      }
      protocol << F("xyPositionerMeasurement +Y:") << measurement << endl;
      return 0;
    }

    // G19: XYPositioner Y2 - Move in -Y until a switch is triggered
    case 19: {
      float measurement;
      if ( xyPositionerTouch(vone->endstops.xyPositionerBack, measurement)
        || moveXY(vone->toolBox.probe, xypos_x_pos, xypos_y_pos)) {
        return -1;
      }
      protocol << F("xyPositionerMeasurement -Y:") << measurement << endl;
      return 0;
    }

    // G20: XYPositioner X1 - Move in +X until a switch is triggered
    case 20: {
      float measurement;
      if ( xyPositionerTouch(vone->endstops.xyPositionerLeft, measurement)
        || moveXY(vone->toolBox.probe, xypos_x_pos, xypos_y_pos)) {
        return -1;
      }
      protocol << F("xyPositionerMeasurement +X:") << measurement << endl;
      return 0;
    }

    // G21: XYPositioner X2 - Move in -X until switch triggered
    case 21: {
      float measurement;
      if ( xyPositionerTouch(vone->endstops.xyPositionerRight, measurement)
        || moveXY(vone->toolBox.probe, xypos_x_pos, xypos_y_pos)) {
          return -1;
      }
      protocol << F("xyPositionerMeasurement -X:") << measurement << endl;
      return 0;
    }

    // G24 - Test the zMIN endstop trigger position
    // move to impossible position, and report where limit switch triggered.
    case 24: {
      vone->stepper.endstopMonitor.ignoreZSwitch(false);

      feedrate = homing_feedrate[Z_AXIS]/(6);
      // move down until you find the bed
      float zPosition = -10;
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);
      protocol << F("Z: ") << (current_position[Z_AXIS] * 1000) << endl;

      vone->stepper.endstopMonitor.ignoreZSwitch();
    }
    return 0;

    // G25 - Test the xAxis endstop trigger position
    // move to impossible position, and report where limit switch triggered.
    case 25: {
      // move down until you find the bed
      feedrate = homing_feedrate[X_AXIS]/(6);
      float xPosition = -10;
      plan_buffer_line(xPosition, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      current_position[X_AXIS] = st_get_position_mm(X_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);
      protocol << F("X: ") << (current_position[X_AXIS] * 1000) << endl;
    }
    return 0;

    // G26 - Test the yAxis endstop trigger position
    // move to impossible position, and report where limit switch triggered.
    case 26: {
      // move down until you find the bed
      feedrate = homing_feedrate[Y_AXIS]/(6);
      float yPosition = -10;
      plan_buffer_line(current_position[X_AXIS], yPosition, current_position[Z_AXIS], current_position[E_AXIS], feedrate/60);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      current_position[Y_AXIS] = st_get_position_mm(Y_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);
      protocol << F("Y: ") << (current_position[Y_AXIS] * 1000) << endl;
    }
    return 0;

    // G27 - Test the zMAX trigger position
    // move to impossible position, and report where limit switch triggered.
    case 27: {
      // move down until you find the bed
      feedrate = homing_feedrate[Z_AXIS]/(6);
      float zPosition = 30;
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);
      protocol << F("Z: ") << (current_position[Z_AXIS] * 1000) << endl;
    }
    return 0;

    // G28 - Home X and Y normally, home Z to the top (legacy code relies on this behavior)
    case 28: {
      bool home_all = !(code_seen('X') || code_seen('Y') || code_seen('Z'));

      vone->toolBox.currentTool().resetPreparations();
      vone->stepper.resume();

      if (home_all || code_seen('Z')) {
        setHomedState(Z_AXIS, 0);
        raise();

        // Set the position so that we can process absolute movements (e.g. G1)
        current_position[Z_AXIS] = Z_MAX_POS;
        plan_set_position(
          current_position[ X_AXIS ],
          current_position[ Y_AXIS ],
          current_position[ Z_AXIS ],
          current_position[ E_AXIS ]
        );
      }

      if (home_all || code_seen('X')) {
        setHomedState(X_AXIS, 0);
      }

      if (home_all || code_seen('Y')) {
        setHomedState(Y_AXIS, 0);
      }

      rawHome(
        vone->toolBox.nullTool,
        home_all || code_seen('X'),
        home_all || code_seen('Y'),
        false
      );

      return 0;
    }

    // G30 - Single Z Probe, probes bed at current XY location.
    case 30: {
      float measurement;
      if (vone->toolBox.probe.probe(measurement, useDefaultFeedrate, NoRetract, 1, 1)) {
        return -1;
      }

      // Output position
      protocol
        << F("probeMeasurement")
        << F(" x:") << current_position[X_AXIS]
        << F(" y:") << current_position[Y_AXIS]
        << F(" z:") << measurement
        << endl;
      return 0;
    }

    // G31 - Reports the Probe Offset
    case 31: {
      float z_probe_offset;
      if (measureProbeDisplacement(vone->toolBox.probe, z_probe_offset)) {
        return -1;
      }
      protocol
        << F("Probe Offset: ")
        << z_probe_offset * 1000 // TODO: should use formating, not multiplication
        << endl;
      return 0;
    }

    // G33 - Homes the Z axis to the Z-switch.
    case 33:
      // Note: it is safer to assume a dispenser is attached than it is to assume a probe is.
      return homeZ(vone->toolBox.dispenser);

    // G90 - Use Absolute Coordinates
    case 90:
      relative_mode = false;
      return 0;

    // G91 - Use Relative Coordinates
    case 91:
      relative_mode = true;
      return 0;

    // G92 - Set current position to coordinates given
    case 92:
      st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
           if(i == E_AXIS) {
             current_position[i] = code_value();
             plan_set_e_position(current_position[E_AXIS]);
           } else {
             current_position[i] = code_value();
             plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           }
        }
      }
      return 0;

    //-------------------------------------------
    // List Commands
    default:
      log << F("G-Commands") << endl;
      log << F("  non-smart commands. It's up to the user to ensure homing and ") << endl;
      log << F("  calibration are performed, if they are needed.") << endl;
      log << F("") << endl;

      log << F("Movement/Utilities") << endl;
      log << F("  G1 - move to the given position -- G1 X10.3 Y23.4 Z3.3 E32 F3000") << endl;
      log << F("  G2 - clockwise arc from current location to given location with arc center at given offset") << endl;
      log << F("       if no target location given current location is used (so you get a circle) -- G2 I5 will make a 10mm circle at X+5") << endl;
      log << F("  G3 - counter-clockwise arc") << endl;
      log << F("") << endl;

      log << F("Utilities") << endl;
      log << F("  G4 - Dwell for a given duration -- G4 S<seconds> or P<milliseconds>") << endl;
      log << F(" G90 - Use Absolute Coordinates") << endl;
      log << F(" G91 - Use Relative Coordinates") << endl;
      log << F(" G92 - Set current position to coordinates given -- G92 Z3.4") << endl;
      log << F("") << endl;

      log << F("Homing/Probing - deprecated") << endl;
      log << F(" G28 - Home axes -- G28 X, G28 XY, G28") << endl;
      log << F(" G30 - Probe bed at current position") << endl;
      log << F(" G33 - Home to z-switch") << endl;
      log << F("") << endl;
      return 0;
  }
}
