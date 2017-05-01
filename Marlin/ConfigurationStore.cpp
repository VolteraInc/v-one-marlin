#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================




#define EEPROM_OFFSET 200
#define EEPROM_OFFSET_CALIB 50
#define EEPROM_VERSION "V10"

// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.


void Config_StoreCalibration(){

  char ver[4]= "000";
  int i=EEPROM_OFFSET_CALIB;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first
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
  //The write order pisses my OCD off.

  //We think we wrote everything fine, so validate offsets by writing the eeprom version.
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET_CALIB;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Offsets Stored");
}

#ifdef EEPROM_SETTINGS
void Config_StoreSettings()
{
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);
  EEPROM_WRITE_VAR(i,max_feedrate);
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  #ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,zprobe_zoffset);

  // PID settings
  float dummy = 3000.0f;
  EEPROM_WRITE_VAR(i,dummy);
  dummy = 0.0f;
  EEPROM_WRITE_VAR(i,dummy);
  EEPROM_WRITE_VAR(i,dummy);

  // LCD settings
  int lcd_contrast = 32;
  EEPROM_WRITE_VAR(i,lcd_contrast);

  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifndef DISABLE_M503
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
    SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
    SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
    SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
    SERIAL_ECHOPAIR(" Y",max_feedrate[1] );
    SERIAL_ECHOPAIR(" Z", max_feedrate[2] );
    SERIAL_ECHOPAIR(" E", max_feedrate[3]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] );
    SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] );
    SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
    SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M204 S",acceleration );
    SERIAL_ECHOPAIR(" T" ,retract_acceleration);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S",minimumfeedrate );
    SERIAL_ECHOPAIR(" T" ,mintravelfeedrate );
    SERIAL_ECHOPAIR(" B" ,minsegmenttime );
    SERIAL_ECHOPAIR(" X" ,max_xy_jerk );
    SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
    SERIAL_ECHOPAIR(" E" ,max_e_jerk);
    SERIAL_ECHOLN("");
}
#endif

void Config_PrintCalibration(){

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Serial No:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(" M504 S:"); SERIAL_ECHOLN(product_serial_number);

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Offsets:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR(" M505 X:", min_z_x_pos);
    SERIAL_ECHOPAIR(" Y:", min_z_y_pos);
    SERIAL_ECHOPAIR(" I:", xypos_x_pos);
    SERIAL_ECHOPAIR(" J:", xypos_y_pos);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Scaling and Skew (A in radians):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(" M506 X:"); SERIAL_PROTOCOL_F(calib_x_scale, 6);
    SERIAL_ECHOPGM(" Y:"); SERIAL_PROTOCOL_F(calib_y_scale, 6);
    SERIAL_ECHOPGM(" A:"); SERIAL_PROTOCOL_F(atan(calib_tan_theta),6); //We use atan because it preserves the sign.
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Backlash Compensation:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(" M507 X:"); SERIAL_PROTOCOL_F(calib_x_backlash, 6);
    SERIAL_ECHOPGM(" Y:"); SERIAL_PROTOCOL_F(calib_y_backlash, 6);
    SERIAL_ECHOLN("");

}

// Attempts to load from EEPROM, reverts to default if not possible.
void Config_RetrieveCalibration()
{
    int i=EEPROM_OFFSET_CALIB;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,product_serial_number);
        EEPROM_READ_VAR(i,min_z_x_pos);
        EEPROM_READ_VAR(i,min_z_y_pos);
        EEPROM_READ_VAR(i,xypos_x_pos);
        EEPROM_READ_VAR(i,xypos_y_pos);
        EEPROM_READ_VAR(i,calib_x_scale);
        EEPROM_READ_VAR(i,calib_y_scale);
        EEPROM_READ_VAR(i,calib_cos_theta);
        EEPROM_READ_VAR(i,calib_tan_theta);
        EEPROM_READ_VAR(i,calib_x_backlash);
        EEPROM_READ_VAR(i,calib_y_backlash);
    }
    else{
        //Print an error and revert to default.
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Error. EEPROM Offsets missing. Is the unit calibrated?");
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
    }

    // It's possible that our stored backlash is garbage.
    if (calib_x_backlash < 0.0 || calib_x_backlash > 1.0 || isnan(calib_x_backlash)) {
      calib_x_backlash = CALIB_X_BACKLASH;
    }
    if (calib_y_backlash < 0.0 || calib_y_backlash > 1.0 || isnan(calib_y_backlash)) {
      calib_y_backlash = CALIB_Y_BACKLASH;
    }

}



#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);
        EEPROM_READ_VAR(i,max_feedrate);
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);

        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
        reset_acceleration_rates();

        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,minsegmenttime);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        #ifndef ULTIPANEL
        int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
        int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
        #endif
        EEPROM_READ_VAR(i,plaPreheatHotendTemp);
        EEPROM_READ_VAR(i,plaPreheatHPBTemp);
        EEPROM_READ_VAR(i,plaPreheatFanSpeed);
        EEPROM_READ_VAR(i,absPreheatHotendTemp);
        EEPROM_READ_VAR(i,absPreheatHPBTemp);
        EEPROM_READ_VAR(i,absPreheatFanSpeed);
        EEPROM_READ_VAR(i,zprobe_zoffset);

        float Kp,Ki,Kd;
        // do not need to scale PID values as the values in EEPROM are already scaled
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);

        int lcd_contrast;
        EEPROM_READ_VAR(i,lcd_contrast);

        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
    }
    else
    {
        Config_ResetDefault();
    }
    #ifdef EEPROM_CHITCHAT
      Config_PrintSettings();
    #endif
}
#endif

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (short i=0;i<4;i++)
    {
        axis_steps_per_unit[i]=tmp1[i];
        max_feedrate[i]=tmp2[i];
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }

    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();

    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif

SERIAL_ECHO_START;
SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");

}
