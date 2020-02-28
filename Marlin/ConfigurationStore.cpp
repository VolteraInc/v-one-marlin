#include "serial.h"
#include "Marlin.h" // e.g. product_serial_number

#include "planner.h"
#include "src/api/movement/movement.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size) {
  do {
    eeprom_write_byte((unsigned char*)pos, *value);
    pos++;
    value++;
  } while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))

void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size) {
  do {
    *value = eeprom_read_byte((unsigned char*)pos);
    pos++;
    value++;
  } while(--size);
}
#define EEPROM_READ_VAR_SIZE(pos, value, size) _EEPROM_readData(pos, (uint8_t*)&value, size)
#define EEPROM_READ_VAR(pos, value) EEPROM_READ_VAR_SIZE(pos, value, sizeof(value))

//======================================================================================

#define EEPROM_OFFSET 200
#define EEPROM_OFFSET_CALIB 50

// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION_NONE "000"
#define EEPROM_VERSION_V10 "V10"
#define EEPROM_VERSION_V11 "V11"
#define EEPROM_VERSION_V12 "V12"
#define EEPROM_VERSION_V13 "V13"
#define EEPROM_CURRENT_VERSION EEPROM_VERSION_V13

#define SETTINGS_VERSION_NONE "000"
#define SETTINGS_VERSION_V12 "V12"
#define CURRENT_SETTINGS_VERSION SETTINGS_VERSION_V12

void Config_StoreCalibration() {
  // Mark the stored settings as invalid
  char ver[4] = EEPROM_VERSION_NONE;
  int i = EEPROM_OFFSET_CALIB;
  EEPROM_WRITE_VAR(i, ver);

  // Write settings
  EEPROM_WRITE_VAR(i, product_serial_number);
  EEPROM_WRITE_VAR(i, min_z_x_pos);
  EEPROM_WRITE_VAR(i, min_z_y_pos);;
  EEPROM_WRITE_VAR(i, xypos_x_pos);
  EEPROM_WRITE_VAR(i, xypos_y_pos);
  EEPROM_WRITE_VAR(i, calib_x_scale);
  EEPROM_WRITE_VAR(i, calib_y_scale);
  EEPROM_WRITE_VAR(i, calib_cos_theta);
  EEPROM_WRITE_VAR(i, calib_tan_theta);
  EEPROM_WRITE_VAR(i, calib_x_backlash);
  EEPROM_WRITE_VAR(i, calib_y_backlash);
  EEPROM_WRITE_VAR(i, z_switch_type);

  // Now write the version, which marks the stored data as valid
  char ver2[4] = EEPROM_CURRENT_VERSION;
  i = EEPROM_OFFSET_CALIB;
  EEPROM_WRITE_VAR(i, ver2);

  log << F("Calibration Stored") << endl;
}

void Config_ClearStoredSettings() {
  char ver[4]= SETTINGS_VERSION_NONE;
  int i = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i, ver);

  log << F("Cleared stored settings -- defaults will be used") << endl;
}

void Config_StoreSettings() {
  // Mark the stored settings as invalid
  char ver[4]= "000";
  int i = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i, ver);

  // Record settings
  EEPROM_WRITE_VAR(i, axis_steps_per_unit);
  EEPROM_WRITE_VAR(i, max_feedrate);
  EEPROM_WRITE_VAR(i, max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i, acceleration);
  EEPROM_WRITE_VAR(i, retract_acceleration);
  EEPROM_WRITE_VAR(i, minimumfeedrate);
  EEPROM_WRITE_VAR(i, mintravelfeedrate);
  EEPROM_WRITE_VAR(i, minsegmenttime);
  EEPROM_WRITE_VAR(i, max_xy_jerk);
  EEPROM_WRITE_VAR(i, max_z_jerk);
  EEPROM_WRITE_VAR(i, max_e_jerk);

  // Now write the version, which marks the stored data as valid
  char ver2[4] = CURRENT_SETTINGS_VERSION;
  i = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i, ver2);

  log << F("Speed settings stored") << endl;
}

void Config_PrintSettings() {
  log << F("Steps per unit:") << endl;
  log << F("  M92 X") << axis_steps_per_unit[X_AXIS]
      << F(" Y") << axis_steps_per_unit[Y_AXIS]
      << F(" Z") << axis_steps_per_unit[Z_AXIS]
      << F(" E") << axis_steps_per_unit[E_AXIS]
      << endl;

  log << F("Maximum feedrates (mm/s):") << endl;
  log << F("  M203 X") << max_feedrate[X_AXIS]
      << F(" Y") << max_feedrate[Y_AXIS]
      << F(" Z") << max_feedrate[Z_AXIS]
      << F(" E") << max_feedrate[E_AXIS]
      << endl;

  log << F("Maximum Acceleration (mm/s2):") << endl;
  log << F("  M201 X" ) << max_acceleration_units_per_sq_second[X_AXIS]
      << F(" Y") << max_acceleration_units_per_sq_second[Y_AXIS]
      << F(" Z") << max_acceleration_units_per_sq_second[Z_AXIS]
      << F(" E") << max_acceleration_units_per_sq_second[E_AXIS]
      << endl;

  log << F("Acceleration: S=acceleration, T=retract acceleration") << endl;
  log << F("  M204 S") << acceleration
      << F(" T") << retract_acceleration
      << endl;

  log << F("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)") << endl;
  log << F("  M205 S") << minimumfeedrate
      << F(" T") << mintravelfeedrate
      << F(" B") << minsegmenttime
      << F(" X") << max_xy_jerk
      << F(" Z") << max_z_jerk
      << F(" E") << max_e_jerk
      << endl;
}

void Config_PrintCalibration() {
  log << F("Serial No:") << endl;
  log << F(" M504 S:") << product_serial_number
      << endl;

  log << F("Offsets:") << endl;
  log << F(" M505 X:") << min_z_x_pos
      << F(" Y:") << min_z_y_pos
      << F(" I:") << xypos_x_pos
      << F(" J:") << xypos_y_pos
      << endl;

  log << F("Scaling and Skew (A in radians):") << endl;
  log << F(" M506 X:") << calib_x_scale
      << F(" Y:") << calib_y_scale
      << F(" A:") << atan(calib_tan_theta) // We use atan because it preserves the sign.
      << endl;

  log << F("Backlash Compensation:") << endl;
  log << F(" M507 X:") << calib_x_backlash
      << F(" Y:") << calib_y_backlash
      << endl;
}

// Attempts to load from EEPROM, reverts to default if not possible.
void Config_RetrieveCalibration() {
  int i = EEPROM_OFFSET_CALIB;

  // Read version of stored settings
  char stored_ver[4];
  EEPROM_READ_VAR(i, stored_ver);
  log << F("Reading calibration settings, version ") << stored_ver << endl;

  const bool isV10 = strncmp(stored_ver, EEPROM_VERSION_V10, 3) == 0;
  const bool isV11 = strncmp(stored_ver, EEPROM_VERSION_V11, 3) == 0;
  const bool isV12 = strncmp(stored_ver, EEPROM_VERSION_V12, 3) == 0;
  const bool isV13 = strncmp(stored_ver, EEPROM_VERSION_V13, 3) == 0;

  // Bad version, print an error and revert to defaults
  if (!isV13 && !isV12 && !isV11 && !isV10) {
    logWarning << F("EEPROM Offsets missing. Is the unit calibrated?") << endl;
    strcpy(product_serial_number, PRODUCT_SERIAL);
    min_z_x_pos= MIN_Z_X_POS;
    min_z_y_pos= MIN_Z_Y_POS;
    xypos_x_pos= XYPOS_X_POS;
    xypos_y_pos= XYPOS_Y_POS;
    calib_x_scale = CALIB_X_SCALE;
    calib_y_scale = CALIB_Y_SCALE;
    calib_cos_theta = CALIB_COS_THETA;
    calib_tan_theta = CALIB_TAN_THETA;
    calib_x_backlash = CALIB_X_BACKLASH;
    calib_y_backlash = CALIB_Y_BACKLASH;
    z_switch_type = -1;
    return;
  }

  if (isV10) {
    EEPROM_READ_VAR_SIZE(i, product_serial_number, 11);
    // Note: we do not upgrade-and-write the stored values using
    // the new format, yet. This allows users to revert to older
    // firmware without loosing settings. Once enough time
    // has passed we can overwrite.
    // Also if the user stores settings we'll use the latest format,
    // to avoid creating special cases in the write code, for what
    // should be an extremely rare scenario.
  } else {
    EEPROM_READ_VAR(i, product_serial_number);
  }

  EEPROM_READ_VAR(i, min_z_x_pos);
  EEPROM_READ_VAR(i, min_z_y_pos);
  EEPROM_READ_VAR(i, xypos_x_pos);
  EEPROM_READ_VAR(i, xypos_y_pos);
  EEPROM_READ_VAR(i, calib_x_scale);
  EEPROM_READ_VAR(i, calib_y_scale);
  EEPROM_READ_VAR(i, calib_cos_theta);
  EEPROM_READ_VAR(i, calib_tan_theta);
  EEPROM_READ_VAR(i, calib_x_backlash);
  EEPROM_READ_VAR(i, calib_y_backlash);

  if (isV12 || isV11 || isV10) {
    z_switch_type = -1;
  } else {
    EEPROM_READ_VAR(i, z_switch_type);
  }

  // It's possible that our stored backlash is garbage.
  if (calib_x_backlash < 0.0 || calib_x_backlash > 1.0 || isnan(calib_x_backlash)) {
    calib_x_backlash = CALIB_X_BACKLASH;
  }
  if (calib_y_backlash < 0.0 || calib_y_backlash > 1.0 || isnan(calib_y_backlash)) {
    calib_y_backlash = CALIB_Y_BACKLASH;
  }
}

void Config_RetrieveSettings() {
    int i = EEPROM_OFFSET;

    // Read stored version
    char stored_ver[4];
    EEPROM_READ_VAR(i, stored_ver);

    // check version
    if (strncmp(stored_ver, CURRENT_SETTINGS_VERSION, 3) == 0) {
      log << F("Reading speed settings, version ") << stored_ver << endl;

      EEPROM_READ_VAR(i, axis_steps_per_unit);
      EEPROM_READ_VAR(i, max_feedrate);
      EEPROM_READ_VAR(i, max_acceleration_units_per_sq_second);

      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();

      EEPROM_READ_VAR(i, acceleration);
      EEPROM_READ_VAR(i, retract_acceleration);
      EEPROM_READ_VAR(i, minimumfeedrate);
      EEPROM_READ_VAR(i, mintravelfeedrate);
      EEPROM_READ_VAR(i, minsegmenttime);
      EEPROM_READ_VAR(i, max_xy_jerk);
      EEPROM_READ_VAR(i, max_z_jerk);
      EEPROM_READ_VAR(i, max_e_jerk);

      logNotice << F("Using stored values for speed settings") << endl;
    } else {
      const auto isOldConfig = strncmp(stored_ver, SETTINGS_VERSION_NONE, 3) != 0;
      if (isOldConfig) {
        log << F("Reading speed settings, version ") << stored_ver << endl;
        Config_ClearStoredSettings();
      }

      Config_UseDefaultSettings();
    }
}

void Config_UseDefaultSettings() {
  float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  float tmp2[] = DEFAULT_MAX_FEEDRATE;
  long tmp3[] = DEFAULT_MAX_ACCELERATION;
  for (short i = 0; i < 4; ++i) {
    axis_steps_per_unit[i]=tmp1[i];
    max_feedrate[i]=tmp2[i];
    max_acceleration_units_per_sq_second[i]=tmp3[i];
  }

  // steps per sq second need to be updated to agree with the units per sq second
  reset_acceleration_rates();

  acceleration = DEFAULT_ACCELERATION;
  retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
  minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
  minsegmenttime = DEFAULT_MINSEGMENTTIME;
  mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
  max_xy_jerk = DEFAULT_XYJERK;
  max_z_jerk = DEFAULT_ZJERK;
  max_e_jerk = DEFAULT_EJERK;

  log << F("Using hardcoded defaults for speed settings") << endl;
}
