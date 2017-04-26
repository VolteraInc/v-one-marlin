#include "processing.h"
#include "../api/api.h"
#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../temperature.h"
#include "../../temperature_profile.h"
#include "../../ConfigurationStore.h"
#include "../../language.h"
#include "../work/work.h" // pending_temp_change HACK

static uint8_t tmp_extruder;
bool CooldownNoWait = true;
bool target_direction;

bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code){
        case 104:
          SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
          break;
        case 221:
          SERIAL_ECHO(MSG_M221_INVALID_EXTRUDER);
          break;
      }
      SERIAL_ECHOLN(tmp_extruder);
      return true;
    }
  }
  return false;
}

int process_mcode(int command_code) {
  switch(command_code) {
    // M17 - Enable/Power all stepper motors
    case 17:
      enable_x();
      enable_y();
      enable_z();
      enable_e0();
      return 0;

    // M18 - Release motors, or set inactivity timeout
    //       Use to disable steppers until next move, or use S<seconds> to specify an inactivity
    //       timeout, after which the steppers will be disabled.  S0 to disable the timeout.
    case 18:
      if (code_seen('S')) {
        setStepperInactiveDuration(code_value() * 1000);
      } else {
        st_synchronize();
        resetToolPreparations();
        bool disableAll = !(code_seen('X') || code_seen('Y') || code_seen('Z') || code_seen('E'));
        if (disableAll || code_seen('X')) disable_x();
        if (disableAll || code_seen('Y')) disable_y();
        if (disableAll || code_seen('Z')) disable_z();
        if (disableAll || code_seen('E')) disable_e0();
      }
      return 0;

    // M82 - Treat E codes as absolute (default)
    case 82:
      axis_relative_modes[3] = false;
      return 0;

    // M83  - Treat E codes as relative while in Absolute Coordinates (G90) mode
    case 83:
      axis_relative_modes[3] = true;
      return 0;

    // M92 - Set axis_steps_per_unit - same syntax as G92
    case 92:
      for(int8_t i = 0; i < NUM_AXIS; ++i) {
        if(code_seen(axis_codes[i])) {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          } else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      return 0;

    // M93 - Manually control LEDs. Set the RGB LEDs using R[1-255] V[1-255] B[1-255] (uses V instead of G for green)
    case 93:
      // Syntax is M93 R:nnn V:nnn B:nnn (0 <= nnn <= 255)
      // V because vert (using 'G' derails the parser, unsurprisingly)
      // (Call with no arguments to release LEDs)

      if (code_seen('R')) analogWrite(LED_RED_PIN, constrain(code_value(), 0, 255));
      if (code_seen('V')) analogWrite(LED_GREEN_PIN, constrain(code_value(), 0, 255));
      if (code_seen('B')) analogWrite(LED_BLUE_PIN, constrain(code_value(), 0, 255));

      glow_led_override = code_seen('R') || code_seen('V') || code_seen('B');
      return 0;

    // M104 - Set extruder target temp
    case 104:
      if (setTargetedHotend(104)) {
        return -1;
      }
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
      setWatch();
      return 0;;

    // M105 - Read current temp
    case 105 :
      if(setTargetedHotend(105)) {
        return -1;
      }

      SERIAL_PROTOCOLPGM("T:");
      SERIAL_PROTOCOL_F(0.0,1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(0.0,1 );
      #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        SERIAL_PROTOCOLPGM(" B:");
        SERIAL_PROTOCOL_F(degBed(),1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(degTargetBed(),1);
      #endif //TEMP_BED_PIN

      SERIAL_PROTOCOLPGM(" @:");
      #ifdef EXTRUDER_WATTS
        SERIAL_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(tmp_extruder))/127);
        SERIAL_PROTOCOLPGM("W");
      #else
        SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));
      #endif

      SERIAL_PROTOCOLPGM(" B@:");
      #ifdef BED_WATTS
        SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
        SERIAL_PROTOCOLPGM("W");
      #else
        SERIAL_PROTOCOL(getHeaterPower(-1));
      #endif

      #ifdef SHOW_TEMP_ADC_VALUES
        #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
          SERIAL_PROTOCOLPGM("    ADC B:");
          SERIAL_PROTOCOL_F(degBed(),1);
          SERIAL_PROTOCOLPGM("C->");
          SERIAL_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
        #endif
        for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
          SERIAL_PROTOCOLPGM("  T");
          SERIAL_PROTOCOL(cur_extruder);
          SERIAL_PROTOCOLPGM(":");
          SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
          SERIAL_PROTOCOLPGM("C->");
          SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
        }
      #endif

      SERIAL_PROTOCOLLN("");
      return 0;

    case 112:
      quickStop();
      // We can optionally reset the planner to the stepper counts in some axes
      if (code_seen(axis_codes[X_AXIS])) current_position[X_AXIS] = st_get_position_mm(X_AXIS);
      if (code_seen(axis_codes[Y_AXIS])) current_position[Y_AXIS] = st_get_position_mm(Y_AXIS);
      if (code_seen(axis_codes[Z_AXIS])) current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
      if (code_seen(axis_codes[E_AXIS])) current_position[E_AXIS] = st_get_position_mm(E_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      return 0;

    // M114 - Output current position to serial port
    case 114:
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL_F(current_position[X_AXIS], 6);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL_F(current_position[Y_AXIS], 6);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
      SERIAL_PROTOCOLPGM(" E:");
      SERIAL_PROTOCOL_F(current_position[E_AXIS], 6);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL_F(st_get_position_mm(X_AXIS), 6);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL_F(st_get_position_mm(Y_AXIS), 6);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL_F(st_get_position_mm(Z_AXIS), 6);

      SERIAL_PROTOCOLPGM(" Absolute X:");
      SERIAL_PROTOCOL(st_get_position(X_AXIS));
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(st_get_position(Y_AXIS));

      // # of moves queued in buffer
      SERIAL_PROTOCOLPGM(" B:");
      SERIAL_PROTOCOL_F(movesplanned(), DEC);

      // Homing state on each axis
      // 0 = not homed, -1 = homed to min extent, 1 = homed to max extent
      SERIAL_PROTOCOLPGM(" H:");
      SERIAL_PROTOCOL(getHomedState(X_AXIS));
      SERIAL_PROTOCOL(',');
      SERIAL_PROTOCOL(getHomedState(Y_AXIS));
      SERIAL_PROTOCOL(',');
      SERIAL_PROTOCOL(getHomedState(Z_AXIS));

      SERIAL_PROTOCOLLN("");
      return 0;

    // M115 - Capabilities string
    case 115:
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      return 0;

    // M119 - Output Endstop status to serial port
    case 119:
      SERIAL_PROTOCOLLN(MSG_M119_REPORT);
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MIN);
        SERIAL_PROTOCOLLN(READ_PIN(X_MIN) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MAX);
        SERIAL_PROTOCOLLN(READ_PIN(X_MAX) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MIN);
        SERIAL_PROTOCOLLN(READ_PIN(Y_MIN) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
        SERIAL_PROTOCOLLN(READ_PIN(Y_MAX) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MIN);
        SERIAL_PROTOCOLLN(READ_PIN(Z_MIN) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
        SERIAL_PROTOCOLLN(READ_PIN(Z_MAX) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(P_TOP_PIN) && P_TOP_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_P_TOP);
        SERIAL_PROTOCOLLN(READ_PIN(P_TOP) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
        #if defined(P_BOT_PIN) && P_BOT_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_P_BOT);
        SERIAL_PROTOCOLLN(READ_PIN(P_BOT) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(XY_MIN_X_PIN) && XY_MIN_X_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_XY_MIN_X);
        SERIAL_PROTOCOLLN(READ_PIN(XY_MIN_X) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(XY_MAX_X_PIN) && XY_MAX_X_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_XY_MAX_X);
        SERIAL_PROTOCOLLN(READ_PIN(XY_MAX_X) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(XY_MIN_Y_PIN) && XY_MIN_Y_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_XY_MIN_Y);
        SERIAL_PROTOCOLLN(READ_PIN(XY_MIN_Y) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      #if defined(XY_MAX_Y_PIN) && XY_MAX_Y_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_XY_MAX_Y);
        SERIAL_PROTOCOLLN(READ_PIN(XY_MAX_Y) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
      #endif
      return 0;
      //TODO: update for all axis, use for loop

    // M122 - We let the planner know where we are. -  Added by VOLTERA
    case 122: {
      st_synchronize();
      // If no axes are specified, we reset the Z axis (for compat with the old software)
      // Otherwise, we reset the specified axes
      bool z_default = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
      if (code_seen(axis_codes[X_AXIS])) current_position[X_AXIS] = st_get_position_mm(X_AXIS);
      if (code_seen(axis_codes[Y_AXIS])) current_position[Y_AXIS] = st_get_position_mm(Y_AXIS);
      if (code_seen(axis_codes[Z_AXIS]) || z_default) current_position[Z_AXIS]  = st_get_position_mm(Z_AXIS);
      if (code_seen(axis_codes[E_AXIS])) current_position[E_AXIS] = st_get_position_mm(E_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      return 0;
    }

    // M123 - set software endstop(s) to current position - specify axis/es and T for top (max) and B for bottom (min). These are reset upon homing
    case 123:
      if (code_seen(axis_codes[X_AXIS])) (code_seen('T') ? max_pos : min_pos)[X_AXIS] = current_position[X_AXIS];
      if (code_seen(axis_codes[Y_AXIS])) (code_seen('T') ? max_pos : min_pos)[Y_AXIS] = current_position[Y_AXIS];
      if (code_seen(axis_codes[Z_AXIS])) (code_seen('T') ? max_pos : min_pos)[Z_AXIS] = current_position[Z_AXIS];
      return 0;

    // M125 - Output current Probe status to serial port
    case 125:
      SERIAL_PROTOCOLPGM("Probe: "); SERIAL_PROTOCOLLN(toolStateAsString(readToolState(getTool())));
      return 0;

    // M140 - Set bed target temp
    case 140:
      if (code_seen('S')) setTargetBed(code_value());
      return 0;

    // M141 - Append to profile
    //        M141 T240 D3600  => Heat to 240C and hold for 3600 seconds
    case 141: {
      int temperature = 0;
      int duration = 0;
      if (code_seen('T'))
        temperature = code_value();
      if (code_seen('D'))
        duration = code_value();

      // Confirm sensible values were received.
      if (profile_validate_input(temperature, duration)){
        return -1;
      }

      // Add the temperature
      profile_add(temperature, duration);
      return 0;
    }

    // Stop the profile.
    case 142:
      profile_reset();
      return 0;

    // M190 - Wait for bed heater to reach target.
    //        Sxxx Wait for bed current temp to reach target temp. Waits only when heating
    //        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
    case 190: {
#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
      if (code_seen('S')) {
        setTargetBed(code_value());
        CooldownNoWait = true;
      } else if (code_seen('R')) {
        setTargetBed(code_value());
        CooldownNoWait = false;
      }
      auto codenum = millis();

      target_direction = isHeatingBed(); // true if heating, false if cooling
      pending_temp_change = true;

      while ( target_direction ? (isHeatingBed()) : (isCoolingBed() && (CooldownNoWait==false)) ) {
        // Print Temp Reading every 1 second while heating up.
        if((millis() - codenum) > 1000) {
          float tt = degHotend(active_extruder);
          SERIAL_PROTOCOLPGM("T:");
          SERIAL_PROTOCOL(tt);
          SERIAL_PROTOCOLPGM(" E:");
          SERIAL_PROTOCOL((int)active_extruder);
          SERIAL_PROTOCOLPGM(" B:");
          SERIAL_PROTOCOL_F(degBed(),1);
          SERIAL_PROTOCOLLN("");
          codenum = millis();
        }
        periodic_work();
      }
      pending_temp_change = false;
      refresh_cmd_timeout();

#endif
      return 0;
    }

    // M200 - D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
    case 200: {
      float area = .0;
      float radius = .0;
      if(code_seen('D')) {
        radius = (float)code_value() * .5;
        if(radius == 0) {
          area = 1;
        } else {
          area = M_PI * pow(radius, 2);
        }
      } else {
        //reserved for setting filament diameter via UFID or filament measuring device
        return -1;
      }
      tmp_extruder = active_extruder;
      if(code_seen('T')) {
        tmp_extruder = code_value();
        if(tmp_extruder >= EXTRUDERS) {
          SERIAL_ECHO_START;
          SERIAL_ECHO(MSG_M200_INVALID_EXTRUDER);
        }
        SERIAL_ECHOLN(tmp_extruder);
        return 0;
      }
      volumetric_multiplier[tmp_extruder] = 1 / area;
      return 0;
    }

    // M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
    case 201:
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      return 0;

    // M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
    case 203:
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      return 0;

    // M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2
    // also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
    case 204:
      if(code_seen('S')) acceleration = code_value() ;
      if(code_seen('T')) retract_acceleration = code_value() ;
      return 0;

    // M205 - advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
    case 205:
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
      return 0;

    // M220 - S<factor in percent>- set speed factor override percentage
    case 220:
      if(code_seen('S')) {
        feedmultiply = code_value();
      }
      return 0;

    // M221 - S<factor in percent>- set extrude factor override percentage
    case 221: {
      if(code_seen('S')) {
        int tmp_code = code_value();
        if (code_seen('T')) {
          if(setTargetedHotend(221)) {
            return -1;
          }
          extruder_multiply[tmp_extruder] = tmp_code;
        } else {
          extrudemultiply = tmp_code;
        }
      }
    }
    return 0;

#ifdef PIDTEMP
    // M301 - Set PID parameters P I and D
    case 301: {
      if(code_seen('P')) Kp = code_value();
      if(code_seen('I')) Ki = scalePID_i(code_value());
      if(code_seen('D')) Kd = scalePID_d(code_value());

      #ifdef PID_ADD_EXTRUSION_RATE
      if(code_seen('C')) Kc = code_value();
      #endif

      updatePID();
      SERIAL_PROTOCOLPGM(" p:"); SERIAL_PROTOCOL(Kp);
      SERIAL_PROTOCOLPGM(" i:"); SERIAL_PROTOCOL(unscalePID_i(Ki));
      SERIAL_PROTOCOLPGM(" d:"); SERIAL_PROTOCOL(unscalePID_d(Kd));
      #ifdef PID_ADD_EXTRUSION_RATE
      //Kc does not have scaling applied above, or in resetting defaults
      SERIAL_PROTOCOLPGM(" c:"); SERIAL_PROTOCOL(Kc);
      #endif
      SERIAL_PROTOCOLLN("");
      return 0;
    }
#endif //PIDTEMP

    // M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
    case 303: {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
      return 0;
    }

#ifdef PIDTEMPBED
    /// M304 - Set bed PID parameters P I and D
    case 304: {
      if(code_seen('P')) bedKp = code_value();
      if(code_seen('I')) bedKi = scalePID_i(code_value());
      if(code_seen('D')) bedKd = scalePID_d(code_value());

      updatePID();
      SERIAL_PROTOCOLPGM(" p:"); SERIAL_PROTOCOL(bedKp);
      SERIAL_PROTOCOLPGM(" i:"); SERIAL_PROTOCOL(unscalePID_i(bedKi));
      SERIAL_PROTOCOLPGM(" d:"); SERIAL_PROTOCOL(unscalePID_d(bedKd));
      SERIAL_PROTOCOLLN("");
      return 0;
    }
#endif //PIDTEMP


    // M400 - Finish all moves
    case 400:
      st_synchronize();
      return 0;

    // M500 - stores parameters in EEPROM
    case 500:
      Config_StoreSettings();
      return 0;

    // M501 - reads parameters from EEPROM (if you need to reset them after you changed them temporarily).
    case 501:
      Config_RetrieveSettings();
      return 0;

    // M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
    case 502:
      Config_ResetDefault();
      return 0;

    // M503 - print the current settings (from memory not from EEPROM)
    case 503:
      Config_PrintSettings();
      return 0;

    // M504 - Store the Serial Number.
    case 504: {
      if (code_seen('S')) {
        memcpy(product_serial_number, code_value_raw(), sizeof(product_serial_number));
      }

      product_serial_number[14] = '\0'; //Terminate the string.
      Config_StoreCalibration();
      return 0;
    }

    // M505 - Store No. Min X, Min Y, XY Positioner locations.
    case 505: {
      if(code_seen('X')) min_z_x_pos = code_value();
      if(code_seen('Y')) min_z_y_pos = code_value();
      if(code_seen('I')) xypos_x_pos = code_value();
      if(code_seen('J')) xypos_y_pos = code_value();
      Config_StoreCalibration();
      return 0;
    }

    // M506 - stores the axis skew offsets (Kx, Ky, Theta) in EEPROM
    case 506: {
      if(code_seen('X')) calib_x_scale = code_value();
      if(code_seen('Y')) calib_y_scale = code_value();
      if(code_seen('A')){// Read the Axis Skew.
        float theta_rad = code_value(); // Save it, as radians save to EEPROM
        calib_cos_theta = cos(theta_rad);
        calib_tan_theta = tan(theta_rad);
      }
      Config_StoreCalibration();
      return 0;
    }

    // M507 - stores the axis backlash in EEPROM
    case 507: {
      if(code_seen('X')) calib_x_backlash = code_value();
      if(code_seen('Y')) calib_y_backlash = code_value();
      Config_StoreCalibration();
      return 0;
    }

    // M520 - print the current calibration settings, offsets and serial number
    case 520: {
      Config_PrintCalibration();
      return 0;
    }

    // - M906 Get all digital potentiometer values.
    case 906: {
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        SERIAL_PROTOCOLLNPGM("Stepper Driver Currents (Max: 255)");
        SERIAL_PROTOCOLPGM("M907 X:"); SERIAL_PROTOCOL((int)digiPotGetCurrent(X_AXIS));
        SERIAL_PROTOCOLPGM("  Y:"); SERIAL_PROTOCOL((int)digiPotGetCurrent(Y_AXIS));
        SERIAL_PROTOCOLPGM("  Z:"); SERIAL_PROTOCOL((int)digiPotGetCurrent(Z_AXIS));
        SERIAL_PROTOCOLPGM("  E:"); SERIAL_PROTOCOLLN((int)digiPotGetCurrent(E_AXIS));
#endif
      return 0;
    }

    // M907 - Set digital trimpot motor current using axis codes.
    case 907: {
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
      for(int i = 0; i < NUM_AXIS; ++i) {
        if(code_seen(axis_codes[i])) {
          digiPotSetCurrent(i, code_value());
        }
      }
#endif
      return 0;
    }

    //-------------------------------------------
    // TODO: List Commands
    default:
      return 0;
  }
}
