#include "processing.h"

#include "../api/api.h"
#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../planner.h"
#include "../../temperature.h"

#include "../../motion_control.h"

// look here for descriptions of G-codes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208

/// XYPOSITIONER
// G18 - Trigger in +Y
// G19 - Trigger in -Y
// G20 - Trigger in +X
// G21 - Trigger in -X

// AUTOTESTING
// G24 - Test the zMIN endstop trigger position
// G25 - Test the xAxis endstop trigger position
// G26 - Test the yAxis endstop trigger position
// G27 - Test the zMAX trigger position

// G28 - Home all Axis
// G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
// G30 - Single Z Probe, probes bed at current XY location.
// G33 - Homes the Z axis to the bottom Z switch.
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to coordinates given
//
// G1001 - Raise until endstop hit
// G1002 - Move to XY-Positioner
// G1003 - Move to Z-Switch
// G1004 - Relative move (same as G91; G01 <args>; G90)


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

static void s_prepare_move() {
  clamp_to_software_endstops(destination);

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("s_prepare_move");
    SERIAL_ECHOPGM(" X:"); SERIAL_ECHO(destination[ X_AXIS ]);
    SERIAL_ECHOPGM(" Y:"); SERIAL_ECHO(destination[ Y_AXIS ]);
    SERIAL_ECHOPGM(" Z:"); SERIAL_ECHO(destination[ Z_AXIS ]);
    SERIAL_ECHOPGM(" E:"); SERIAL_ECHO(destination[ E_AXIS ]);
    SERIAL_ECHOPGM(" F:"); SERIAL_ECHO(feedrate);
    SERIAL_ECHOPGM("\n");
  }

  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
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
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

int process_gcode(int command_code) {
  switch(command_code) {
    case 0: // G0 -> G1
    case 1: // G1
      if(!IsStopped()) {
        s_get_coordinates(); // For X Y Z E F
        s_prepare_move();
      }
      return 0;

    case 2: // G2  - CW ARC
      if(!IsStopped()) {
        s_get_arc_coordinates();
        s_prepare_arc_move(true);
      }
      return 0;

    case 3: // G3  - CCW ARC
      if(!IsStopped()) {
        s_get_arc_coordinates();
        s_prepare_arc_move(false);
      }
      return 0;

    // G4 dwell
    case 4: {
      unsigned long waitUntil = 0;
      if(code_seen('P')) waitUntil = code_value(); // milliseconds to wait
      if(code_seen('S')) waitUntil = code_value() * 1000; // seconds to wait

      st_synchronize();
      waitUntil += millis();  // keep track of when we started waiting
      while(millis() < waitUntil ){ // TODO: dangerous! could skip work on loop
        manage_heater();
        manage_inactivity();
      }
      return 0;
    }

    // G33 Homes the Z axis to the z-switch.
    case 33:
      // Note: it is safer to assume a dispenser is attached than it is to assume a probe is.
      return homeZ(TOOLS_DISPENSER);

    // G28 Home X and Y normally, home Z to the top (legacy code relies on this behavior)
    case 28: {
      bool home_all = !(code_seen('X') || code_seen('Y') || code_seen('Z'));

      resetToolPreparations();

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

      home(
        TOOLS_NONE,
        home_all || code_seen('X'),
        home_all || code_seen('Y'),
        false
      );

      return 0;
    }

    // G18: XYPositioner Y1 - Move in +Y until a switch is triggered
    case 18: {
      float measurement;
      if ( xyPositionerTouch(TOOLS_PROBE, Y_AXIS, 1, measurement)
        || moveXY(TOOLS_PROBE, xypos_x_pos, xypos_y_pos)) {
        return -1;
      }
      SERIAL_PROTOCOLPGM("xyPositionerMeasurement +Y:"); SERIAL_PROTOCOL_F(measurement, 3);
      SERIAL_PROTOCOLPGM("\n");
      return 0;
    }

    // G19: XYPositioner Y2 - Move in -Y until a switch is triggered
    case 19: {
      float measurement;
      if ( xyPositionerTouch(TOOLS_PROBE, Y_AXIS, -1, measurement)
        || moveXY(TOOLS_PROBE, xypos_x_pos, xypos_y_pos)) {
        return -1;
      }
      SERIAL_PROTOCOLPGM("xyPositionerMeasurement -Y:"); SERIAL_PROTOCOL_F(measurement, 3);
      SERIAL_PROTOCOLPGM("\n");
      return 0;
    }

    // G20: XYPositioner X1 - Move in +X until a switch is triggered
    case 20: {
      float measurement;
      if ( xyPositionerTouch(TOOLS_PROBE, X_AXIS, 1, measurement)
        || moveXY(TOOLS_PROBE, xypos_x_pos, xypos_y_pos)) {
        return -1;
      }
      SERIAL_PROTOCOLPGM("xyPositionerMeasurement +X:"); SERIAL_PROTOCOL_F(measurement, 3);
      SERIAL_PROTOCOLPGM("\n");
      return 0;
    }

    // G21: XYPositioner X2 - Move in -X until switch triggered
    case 21: {
      float measurement;
      if ( xyPositionerTouch(TOOLS_PROBE, X_AXIS, -1, measurement)
        || moveXY(TOOLS_PROBE, xypos_x_pos, xypos_y_pos)) {
          return -1;
      }
      SERIAL_PROTOCOLPGM("xyPositionerMeasurement -X:"); SERIAL_PROTOCOL_F(measurement, 3);
      SERIAL_PROTOCOLPGM("\n");
      return 0;
    }

    // Test the zAxis - move to impossible position, and report where limit switch triggered.
    case 24: {
      feedrate = homing_feedrate[Z_AXIS]/(6);
      // move down until you find the bed
      float zPosition = -10;
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);
      SERIAL_PROTOCOLPGM("Z: ");
      SERIAL_PROTOCOL(float(current_position[Z_AXIS]*1000));
      SERIAL_PROTOCOLPGM("\n");
    }
    return 0;

    // Test the xAxis - move to impossible position, and report where limit switch triggered.
    case 25: {
      // move down until you find the bed
      feedrate = homing_feedrate[X_AXIS]/(6);
      float xPosition = -10;
      plan_buffer_line(xPosition, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      current_position[X_AXIS] = st_get_position_mm(X_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);
      SERIAL_PROTOCOLPGM("X: ");
      SERIAL_PROTOCOL(float(current_position[X_AXIS]*1000));
      SERIAL_PROTOCOLPGM("\n");
    }
    return 0;

    // Test the yAxis - move to impossible position, and report where limit switch triggered.
    case 26: {
      // move down until you find the bed
      feedrate = homing_feedrate[Y_AXIS]/(6);
      float yPosition = -10;
      plan_buffer_line(current_position[X_AXIS], yPosition, current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      current_position[Y_AXIS] = st_get_position_mm(Y_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);
      SERIAL_PROTOCOLPGM("Y: ");
      SERIAL_PROTOCOL(float(current_position[Y_AXIS]*1000));
      SERIAL_PROTOCOLPGM("\n");
    }
    return 0;

    //Test the zAxis - move to impossible position, and report where limit switch triggered.
    case 27: {
      // move down until you find the bed
      feedrate = homing_feedrate[Z_AXIS]/(6);
      float zPosition = 30;
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);
      SERIAL_PROTOCOLPGM("Z: ");
      SERIAL_PROTOCOL(float(current_position[Z_AXIS]*1000));
      SERIAL_PROTOCOLPGM("\n");
    }
    return 0;

    // G30 Single Z Probe
    case 30: {
      float measurement;
      if (probe(TOOLS_PROBE, measurement, NoRetract)) {
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

    // G31 Reports the Probe Offset
    case 31: {
      float z_probe_offset;
      if (measureProbeDisplacement(TOOLS_PROBE, z_probe_offset)) {
        return -1;
      }
      SERIAL_PROTOCOLPGM("Probe Offset: "); SERIAL_PROTOCOL(z_probe_offset * 1000); // TODO: should use SERIAL_PROTOCOL_F instead of *1000
      SERIAL_PROTOCOLPGM("\n");
      return 0;
    }

    case 90: // G90
      relative_mode = false;
      return 0;

    case 91: // G91
      relative_mode = true;
      return 0;

    case 92: // G92
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
      // TODO: List Commands
      default:
        return 0;
  }
}
