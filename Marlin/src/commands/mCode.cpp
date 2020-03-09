#include "processing.h"
#include "../api/api.h"
#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../planner.h"
#include "../../temperature_profile.h"
#include "../../ConfigurationStore.h"
#include "../work/work.h" // pending_temp_change HACK
#include "../../macros.h"
#include "../../version.h"
#include "../vone/VOne.h"
#include "../vone/stepper/digipots.h"
#include "../vone/stepper/trinamicMotors.h"

bool CooldownNoWait = true;
bool target_direction;

int process_mcode(int command_code) {
  switch(command_code) {
    // M17 - Enable/Power all stepper motors
    case 17:
      vone->motors.on();
      return 0;

    // M18 - Release motors, or set inactivity timeout
    //       Use to disable steppers until next move, or use S<seconds> to specify an inactivity
    //       timeout, after which the steppers will be disabled.  S0 to disable the timeout.
    case 18:
      if (code_seen('S')) {
        setStepperInactiveDuration(code_value() * 1000);
      } else {
        st_synchronize();
        vone->toolBox.currentTool().resetPreparations();
        bool disableAll = !(code_seen('X') || code_seen('Y') || code_seen('Z') || code_seen('E'));
        if (disableAll || code_seen('X')) vone->motors.xAxis.off();
        if (disableAll || code_seen('Y')) vone->motors.yAxis.off();
        if (disableAll || code_seen('Z')) vone->motors.zAxis.off();
        if (disableAll || code_seen('E')) vone->motors.eAxis.off();
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
      // (Call with no arguments to release LEDs)
      // Note: 'G' _use_ to break the gcode parser, so we accepted V (as in vert).
      // we continue to support V for backkwards compat.
      return overrideLeds(
        code_seen('R') ? constrain(code_value(), 0, 255) : 0,
        (code_seen('G') || code_seen('V')) ? constrain(code_value(), 0, 255) : 0,
        code_seen('B') ? constrain(code_value(), 0, 255) : 0,
        code_seen('P') ? code_value() : 30
      );
      return 0;

    // M105 - Read current temp
    // TODO: not sure if this output is used in manufacturing scripts.
    // I'd like to trim it down
    case 105 :
      protocol
        << F("T:0.0")
        << F(" /0.0")
        << F(" B:") << FloatWithFormat(vone->heater.currentTemperature(), 1)
        << F(" /") << FloatWithFormat(vone->heater.targetTemperature(), 1)
        << F(" @:0")
        << F(" B@:") << (vone->heater.isHeating() ? 127 : 0)
        << endl;
      return 0;

    case 112:
      quickStop();
      // Reset the planner to the stepper counts in some axes
      return vone->stepper.resyncWithStepCount(
        code_seen('X'),
        code_seen('Y'),
        code_seen('Z'),
        code_seen('E')
      );

    // M114 - Output current position to serial port
    case 114:
      protocol
        << F("X:") << current_position[X_AXIS]
        << F(" Y:") << current_position[Y_AXIS]
        << F(" Z:") << current_position[Z_AXIS]
        << F(" E:") << current_position[E_AXIS]

        << F(" Count X: ") << st_get_position_mm(X_AXIS)
        << F(" Y:") << st_get_position_mm(Y_AXIS)
        << F(" Z:") << st_get_position_mm(Z_AXIS)

        << F(" Absolute X:") << st_get_position(X_AXIS)
        << F(" Y:") << st_get_position(Y_AXIS)
        << F(" Z:") << st_get_position(Z_AXIS)

        // # of moves queued in buffer
        << F(" B:") << (int)movesplanned()

        // Homing state on each axis
        // 0 = not homed, -1 = homed to min extent, 1 = homed to max extent
        << F(" H:") << getHomedState(X_AXIS) << ',' << getHomedState(Y_AXIS) << ',' << getHomedState(Z_AXIS)
        << endl;
      return 0;

    // M115 - Capabilities string
    case 115:
      protocol << F("V-One by Voltera (www.voltera.io). Firmware: ") << VERSION_STRING << endl;
      return 0;

    // M119 - Output Endstop status to serial port
    case 119:
      vone->endstops.deprecated_outputStatus();
      return 0;

    // M122 - We let the planner know where we are
    case 122: {
      st_synchronize();
      bool hasX = code_seen('X');
      bool hasY = code_seen('Y');
      bool hasE = code_seen('E');

      // If no axes are specified, we reset the Z axis (for compat with the old software)
      // NOTE: this 'old software' may no longer exists
      bool zDefault = !hasX && !hasY && !hasE;
      bool hasZ = zDefault || code_seen('Z');
      return vone->stepper.resyncWithStepCount(hasX, hasY, hasZ, hasE);
    }

    // M123 - set software endstop(s) to current position - specify axis/es and T for top (max) and B for bottom (min). These are reset upon homing
    case 123:
      if (code_seen(axis_codes[X_AXIS])) (code_seen('T') ? max_pos : min_pos)[X_AXIS] = current_position[X_AXIS];
      if (code_seen(axis_codes[Y_AXIS])) (code_seen('T') ? max_pos : min_pos)[Y_AXIS] = current_position[Y_AXIS];
      if (code_seen(axis_codes[Z_AXIS])) (code_seen('T') ? max_pos : min_pos)[Z_AXIS] = current_position[Z_AXIS];
      return 0;

    // M125 - Output current Probe status to serial port
    case 125: {
      auto& probe = vone->toolBox.probe;
      auto prefix = F("Probe: ");
      if (probe.detached()) {
        protocol << prefix << F("Not Mounted") << endl;
      } else if (probe.readAnalogTriggered()) {
        protocol << prefix << F("Triggered") << endl;
      } else {
        protocol << prefix << F("Probe Mounted") << endl;
      }
      return 0;
    }

    // M140 - Set bed target temp
    case 140:
      if (code_seen('S')) {
        vone->heater.setTargetTemperature(code_value());
      }
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

      // Add the temperature
      return profile_add(temperature, duration);
    }

    // Stop the profile.
    case 142:
      profile_reset();
      return 0;

    // M190 - Wait for bed heater to reach target.
    //        Sxxx Wait for bed current temp to reach target temp. Waits only when heating
    //        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
    case 190: {
      if (code_seen('S')) {
        vone->heater.setTargetTemperature(code_value());
        CooldownNoWait = true;
      } else if (code_seen('R')) {
        vone->heater.setTargetTemperature(code_value());
        CooldownNoWait = false;
      }
      auto codenum = millis();

      target_direction = vone->heater.isHeating(); // true if heating, false if cooling
      pending_temp_change = true;

      while ( target_direction ? (vone->heater.isHeating()) : (vone->heater.isCooling() && (CooldownNoWait==false)) ) {
        // Print Temp Reading every 1 second while heating up.
        if ((millis() - codenum) > 1000) {
          protocol << F("T:0 E:0 B:") << FloatWithFormat(vone->heater.currentTemperature(), 1) << endl;
          codenum = millis();
        }
        periodic_work();
      }
      pending_temp_change = false;
      refresh_cmd_timeout();

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
      volumetric_multiplier = 1 / area;
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
    case 221:
      if (code_seen('S')) {
        extrudemultiply = code_value();
      }
      return 0;

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

    // M502 - reverts to the default "factory settings".
    //        include an 'S' to clear stored settings too.
    case 502: {
      Config_UseDefaultSettings();

      const auto save = code_seen('S');
      if (save) {
        Config_ClearStoredSettings();
      }
      return 0;
    }

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

    // 600 - Infield Upgrades
    // -------------------

    // M601 - z-switch replacement
    case 601:
      if (code_seen('S')) {
        auto zSwitchType = ZSwitch::toType(
          static_cast<int>(code_value_long())
        );

        log
          << F("Setting Z-Switch type to ")
          << ZSwitch::typeName(zSwitchType)
          << F(", reboot for change to take effect")
          << endl;

        z_switch_type = static_cast<int>(zSwitchType);
        Config_StoreCalibration();
      }
      return 0;

#ifdef TRINAMIC_MOTORS

    //  M900 X20 -> Set current in miliamps and get current value.
    case 900: {
      protocol << F("Stepper - Set current in milliamps (i.e - M900 X200 Y300)") << endl;
      if (code_seen(X_AXIS)) { trinamicSetCurrent(X_AXIS, code_value()); }
      if (code_seen(Y_AXIS)) { trinamicSetCurrent(Y_AXIS, code_value()); }
      if (code_seen(Z_AXIS)) { trinamicSetCurrent(Z_AXIS, code_value()); }
      if (code_seen(E_AXIS)) { trinamicSetCurrent(E_AXIS, code_value()); }

      protocol << F("Stepper - Current Values (Max: 32)") << endl;
      protocol
        << F("M900 X:") << trinamicGetCurrentScaling(X_AXIS)
        << F("  Y:") << trinamicGetCurrentScaling(Y_AXIS)
        << F("  Z:") << trinamicGetCurrentScaling(Z_AXIS)
        << F("  E:") << trinamicGetCurrentScaling(E_AXIS)
        << endl;
      return 0;
    }

    case 901: {
      protocol << F("Stepper - Actual TStep: (32 bit)") << endl;
      protocol
        << F("M901 X:") << trinamicGetTStep(X_AXIS)
        << F("  Y:") << trinamicGetTStep(Y_AXIS)
        << F("  Z:") << trinamicGetTStep(Z_AXIS)
        << F("  E:") << trinamicGetTStep(E_AXIS)
        << endl;
      return 0;
    }

  case 902: {
    protocol << F("Stepper - Set Stallguard value (i.e - M902 X0 Y60)") << endl;
    if (code_seen(X_AXIS)) { trinamicSetSG(X_AXIS, code_value()); }
    if (code_seen(Y_AXIS)) { trinamicSetSG(Y_AXIS, code_value()); }
    if (code_seen(Z_AXIS)) { trinamicSetSG(Z_AXIS, code_value()); }
    if (code_seen(E_AXIS)) { trinamicSetSG(E_AXIS, code_value()); }

    protocol << F("Stepper - Stall Guard Values (-64...0... 63)") << endl;
    protocol
      << F("M902 X:") << trinamicGetSG(X_AXIS)
      << F("  Y:") << trinamicGetSG(Y_AXIS)
      << F("  Z:") << trinamicGetSG(Z_AXIS)
      << F("  E:") << trinamicGetSG(E_AXIS)
      << endl;
    return 0;
  }

  case 903: {
    protocol << F("Stepper - GSTAT") << endl;
    protocol
      << F("M903 X:") << trinamicGetGStat(X_AXIS)
      << F("  Y:") << trinamicGetGStat(Y_AXIS)
      << F("  Z:") << trinamicGetGStat(Z_AXIS)
      << F("  E:") << trinamicGetGStat(E_AXIS)
      << endl;
    return 0;
  }

  case 904: {
    protocol << F("Stepper - DRVSTATUS") << endl;
    protocol
      << F("M904 X:") << trinamicGetDRVSTATUS(X_AXIS)
      << F("  Y:") << trinamicGetDRVSTATUS(Y_AXIS)
      << F("  Z:") << trinamicGetDRVSTATUS(Z_AXIS)
      << F("  E:") << trinamicGetDRVSTATUS(E_AXIS)
      << endl;
    return 0;
  }

#else

    // - M906 Get all digital potentiometer values.
    case 906: {
      protocol << F("Stepper Driver Currents (Max: 255)") << endl;
      protocol
        << F("M907 X:") << (int)digiPotGetCurrent(X_AXIS)
        << F("  Y:") << (int)digiPotGetCurrent(Y_AXIS)
        << F("  Z:") << (int)digiPotGetCurrent(Z_AXIS)
        << F("  E:") << (int)digiPotGetCurrent(E_AXIS)
        << endl;
      return 0;
    }

    // M907 - Set digital trimpot motor current using axis codes.
    case 907: {
      for(int i = 0; i < NUM_AXIS; ++i) {
        if(code_seen(axis_codes[i])) {
          digiPotSetCurrent(i, code_value());
        }
      }
      return 0;
    }

#endif // TRINAMIC_MOTORS

    //-------------------------------------------
    // List Commands
    default:
      log << F("M-Commands") << endl;
      log << F("  Configuration, temperature control and Utility commands") << endl;
      log << endl;

      log << F("Utilities") << endl;
      log << F("  M400 - Finish all moves") << endl;
      log << F("  M93  - Manually control LEDs. Set the RGB LEDs using R[1-255] V[1-255] B[1-255] (uses V instead of G for green)") << endl;
      log << endl;

      log << F("Temperature") << endl;
      log << F("  M105 - Output current temperature") << endl;
      log << F("  M141 - Append a temperature and duration to the heating profile (max 10 commands) -- M141 T200 D60") << endl;
      log << F("  M142 - Stop heating and discard heating profile") << endl;
      log << endl;

      log << F("Status") << endl;
      log << F("  M114 - Output status (planner and stepper position, queued moves, etc)") << endl;
      log << F("  M115 - Output version string") << endl;
      log << F("  M119 - Output limit switch status") << endl;
      log << F("  M125 - Output current Probe status to serial port") << endl;
      log << endl;

      log << F("Motors") << endl;
      log << F("  M17  - Enable (Power) all stepper motors") << endl;
      log << F("  M18  - Disable motors until next move -- M18 X Y Z E or M18 for all") << endl;
      log << F("         set inactivity timeout in seconds -- M18 S60, S0 to disable") << endl;
      log << endl;

      log << F("Configuration") << endl;
      log << F("  M500 - write configuration to EEPROM") << endl;
      log << F("  M501 - read (ie. reset) configuration from EEPROM") << endl;
      log << F("  M502 - revert to factory defaults (still need to write to EEPROM)") << endl;
      log << F("  M503 - output configuration") << endl;
      log << endl;

      log << F("Infield Upgrades") << endl;
      log << F("  M601 - z-switch replacement (S0 original, S1 stronger spring, S-1 use default) -- M601 S1") << endl;
      log << endl;


      log << F("Internal use") << endl;
      log << F("  M92  - Set axis steps per unit - same syntax as G92") << endl;
      log << F("  M520 - output calibrated positions and factors") << endl;
      log << F("  M906 - output driver current settings") << endl;
      log << F("  M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)") << endl;
      log << F("  M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec") << endl;
      log << F("  M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2") << endl;
      log << F("         also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate") << endl;
      log << F("  M205 - advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk") << endl;
      log << endl;
      return 0;
  }
}
