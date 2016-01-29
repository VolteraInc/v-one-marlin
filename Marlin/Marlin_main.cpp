/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#define VERSION_STRING  "1.0.0"

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

// M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
//        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
//        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
//        The '#' is necessary when calling from within sd files, as it stops buffer prereading
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M93  - Set the RGB LEDs using R[1-255] V[1-255] B[1-255] (uses V instead of G for green)
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
// M114 - Output current position to serial port
// M115 - Capabilities string
// M119 - Output Endstop status to serial port
// M125 - Output current Probe status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
// M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homing offset
// M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
// M240 - Trigger a camera to take a photograph
// M250 - Set LCD contrast C<contrast value> (value 0..63)
// M300 - Play beep sound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M401 - Lower z-probe if present
// M402 - Raise z-probe if present
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from EEPROM)
// M504 - stores offsets and serial number in EEPROM
// M505 - print the current offsets from EEPROM
// M506 - print the current serial number from EEPROM
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M665 - set delta configurations
// M666 - set delta endstop adjustment
// M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
// M906 Get all digital potentiometer values.
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
bool homing_axis = false;
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
int extruder_multiply[EXTRUDERS] = {100};
float volumetric_multiplier[EXTRUDERS] = {1.0};
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
#ifdef DELTA
float endstop_adj[3]={0,0,0};
#endif
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
#define HOMED_NONE 0
signed char axis_homed_state[3] = {HOMED_NONE, HOMED_NONE, HOMED_NONE};
float zprobe_zoffset;

uint8_t active_extruder = 0;
int fanSpeed=0;

bool glow_led_override = false;
bool glow_force_green = false; // For taking pictures of the printer without a PC attached.
bool override_p_bot = false;
float min_z_x_pos;
float min_z_y_pos;
float xypos_x_pos;
float xypos_y_pos;
char product_serial_number[11] = PRODUCT_SERIAL;

//===========================================================================
//=============================Private Variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_serial_rx = 0;
static unsigned long previous_millis_active_cmd = 0;
// After this long without serial traffic *and* no movement, everything shuts down
static unsigned long max_no_serial_no_movement_time = 60000;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;


bool Stopped=false;

bool CooldownNoWait = true;
bool pending_temp_change = false;
bool target_direction;

static bool ensure_homed_enable = false;
static bool ensure_homed_preemptive_all_axis = false;

//Insert variables if CHDK is defined
#ifdef CHDK
unsigned long chdkHigh = 0;
boolean chdkActive = false;
#endif

//===========================================================================
//=============================Routines======================================
//===========================================================================

void checkBufferEmpty();

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

extern "C"{
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happens
    strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void setup()
{
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  /*SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(VERSION_STRING);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif
  #endif*/
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  Config_RetrieveCalibration();

  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!

  glow_force_green = READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING;
}


void loop()
{
  if(buflen < (BUFSIZE-1))
    get_command();
  if(buflen)
  {
    previous_millis_serial_rx = millis();
    process_commands();
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  checkBufferEmpty();
}

void get_command()
{
  while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strchr(cmdbuffer[bufindw], 'N') != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            SERIAL_ERRORLN(gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              SERIAL_ERRORLN(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
              ACK_CMD
            }
            else {
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
            }
            break;
          default:
            break;
          }

        }
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}

float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

static void axis_is_at_home(int axis, int flip) {

  current_position[axis] = base_home_pos(axis) + add_homeing[axis];
  if (flip)
  current_position[axis] = base_min_pos(axis) + add_homeing[axis];
  min_pos[axis] =          base_min_pos(axis) + add_homeing[axis];
  max_pos[axis] =          base_max_pos(axis) + add_homeing[axis];
}

static void run_z_probe() {

    feedrate = homing_feedrate[Z_AXIS]*5;

    // move down until you find the bed
    float zPosition = -10;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // we have to let the planner know where we are right now as it is not where we said to go.
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

    // // move up the retract distance
    // zPosition += home_retract_mm(Z_AXIS);
    // plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    // st_synchronize();

    // // move back down slowly to find bed
    // feedrate = homing_feedrate[Z_AXIS]/6.0; //VOLTERA (ORIGINALLY 2)
    // zPosition -= home_retract_mm(Z_AXIS) * 2;
    // plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    // st_synchronize();

    current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
    // make sure the planner knows where we are as it may be a bit different than we last said to move to
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

static void setup_for_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    previous_millis_active_cmd = millis();
    enable_endstops(true);
}

static void clean_up_after_endstop_move() {
#ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
#endif
    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    previous_millis_active_cmd = millis();
}

static void homeaxis(int axis, bool flip) {

    homing_axis = true; // Raise flag to let planner know we are homing and axis so it ignores skew adjustments.
    int axis_home_dir = home_dir(axis);

    if (flip){
      axis_home_dir = axis_home_dir*-1 ;
    }

    if (!axis_homed_state[axis]) current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

    // If we already know roughly where we are, don't go too far past the known extent
    // (e.g. if tool isn't mounted and we're trying to home z bottom after having homed z top)
    // Only do this in Z because no other axis has double endstops
    bool soft_limit_home = axis == Z_AXIS;
    destination[axis] = soft_limit_home && axis_homed_state[axis] ? (axis_home_dir > 0 ? max_pos : min_pos)[axis] + axis_home_dir : 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    endstops_hit_on_purpose(); // Clear endstop flags

    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    if (!didHitEndstops()) {
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM(" home ");
      SERIAL_ERROR(axis_codes[axis]);
      SERIAL_ERRORLNPGM(" failed - no limit hit");
      // We don't actually know if this is where we are (maybe it stalled)
      // ...so this error should probably be followed with another homing attempt
      current_position[axis] = destination[axis];
      return;
    }

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis]/(6);
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
    axis_is_at_home(axis, flip);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_homed_state[axis] = axis_home_dir;

    homing_axis = false; //Lower flag to indicate homing was finished. It doesn't matter if it was not sucessful.

}

void refresh_cmd_timeout(void)
{
  previous_millis_active_cmd = millis();
}


void ensure_homed(bool need_x, bool need_y, bool need_z) {
  static float saved_destination[NUM_AXIS];
  // We don't want to preemptive home if it's just the e axis (i.e. not xyz) moving
  if (ensure_homed_preemptive_all_axis && (need_x || need_y || need_z)) {
    need_x = need_y = need_z = true;
  }

  bool home_x = need_x && !axis_homed_state[X_AXIS];
  bool home_y = need_y && !axis_homed_state[Y_AXIS];
  bool home_z = need_z && !axis_homed_state[Z_AXIS];

  if (!home_x && !home_y && !home_z) return;

  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;
  enable_endstops(true);
  memcpy(saved_destination, destination, sizeof(saved_destination));
  memcpy(destination, current_position, sizeof(current_position));

  feedrate = 0.0;

  if (home_z) homeaxis(Z_AXIS, 0);
  if (home_y) homeaxis(Y_AXIS, 0);
  if (home_x) homeaxis(X_AXIS, 0);

  #ifdef ENDSTOPS_ONLY_FOR_HOMING
  enable_endstops(false);
  #endif
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
  endstops_hit_on_purpose();

  for (int i = 0; i < NUM_AXIS; ++i)
  {
    if (code_seen(axis_codes[i])) destination[i] = saved_destination[i];
  }

  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
}

bool ensure_requested_homing(void) {
  // You can specify H#,#,# on a movement command to ensure that the printer is homed to a particular endstop before moving
  // If it isn't, it'll bail (it *won't* implicitly home)
  // Note that the parsing is super sketch, probably best not to pass malformed syntax
  // This exists soley because the Z axis coordinate system shifts ~15mm depending on whether we're homed to bottom or top Z
  if (code_seen('H')) {
    signed char home_x = code_value_long();
    strchr_pointer += (home_x < 0 ? 2 : 1) + 1;
    signed char home_y = code_value_long();
    strchr_pointer += (home_y < 0 ? 2 : 1) + 1;
    signed char home_z = code_value_long();

    if ((home_x && axis_homed_state[X_AXIS] != home_x) || (home_y && axis_homed_state[Y_AXIS] != home_y) || (home_z && axis_homed_state[Z_AXIS] != home_z)) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(" move failed - printer not homed as requested");
      return false;
    }
  }
  return true;
}

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
        //ClearToSend();
        return;
      }
      //break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4: // G4 dwell
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_active_cmd = millis();
      while(millis()  < codenum ){
        manage_heater();
        manage_inactivity();
      }
      break;

    case 33: // G33 Homes the Z axis to the other switch.

      saved_feedrate = feedrate;
      previous_millis_active_cmd = millis();
      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;

      homeaxis(Z_AXIS,1);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      previous_millis_active_cmd = millis();
      endstops_hit_on_purpose();
      break;


    case 28: //G28 Home all Axis one at a time

      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_active_cmd = millis();

      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;


      home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));

      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        #ifdef VOLTERA
        homeaxis(Z_AXIS,0);
        #endif
      }
      #endif

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        homeaxis(Y_AXIS,0);
      }

      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      {
        homeaxis(X_AXIS,0);
      }

      if(code_seen(axis_codes[X_AXIS]))
      {
        if(code_value_long() != 0) {
          current_position[X_AXIS]=code_value()+add_homeing[0];
        }
      }

      if(code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Y_AXIS]=code_value()+add_homeing[1];
        }
      }

      if(code_seen(axis_codes[Z_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Z_AXIS]=code_value()+add_homeing[2];
        }
      }

      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_active_cmd = millis();
      endstops_hit_on_purpose();
      break;

    case 18: // G18:  XYPositioner Y1 - Move in +Y until a switch is triggered
      {
          enable_endstops(true);
          // move in +Y until a switch is triggered
          feedrate = homing_feedrate[X_AXIS]/(2);
          float yPosition = current_position[Y_AXIS] + 10;
          plan_buffer_line(current_position[X_AXIS], yPosition, current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();

          // we have to let the planner know where we are right now as it is not where we said to go.
          current_position[Y_AXIS] = st_get_position_mm(Y_AXIS);
          SERIAL_PROTOCOLPGM("xypos Y: ");
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);

          if (!didHitEndstops()) {
            SERIAL_ERROR_START;
            SERIAL_ERRORLNPGM(" xypos failed - no limit hit");
          }

          // Output the trigger position in Y in microns.
          SERIAL_PROTOCOLLN(float(current_position[Y_AXIS]*1000));
          enable_endstops(false);
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS]-2.0, current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          enable_endstops(true);
    }
      break;

      case 19: // G19: XYPositioner Y2 - Move in -Y until a switch is triggered
      {
          enable_endstops(true);
          // move in -Y until a switch is triggered
          feedrate = homing_feedrate[Y_AXIS]/(2);
          float yPosition = current_position[Y_AXIS] - 10;
          plan_buffer_line(current_position[X_AXIS], yPosition, current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();

          // we have to let the planner know where we are right now as it is not where we said to go.
          current_position[Y_AXIS] = st_get_position_mm(Y_AXIS);
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);

          if (!didHitEndstops()) {
            SERIAL_ERROR_START;
            SERIAL_ERRORLNPGM(" xypos failed - no limit hit");
          }

          SERIAL_PROTOCOLPGM("xypos Y: ");
          SERIAL_PROTOCOLLN(float(current_position[Y_AXIS]*1000));
          enable_endstops(false);
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS]+2.0, current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          enable_endstops(true);
      }
      break;

    case 20: // G20: XYPositioner X1 - Move in +X until a switch is triggered
      {
          enable_endstops(true);
          // move in +X until a switch is triggered
          feedrate = homing_feedrate[X_AXIS]/(2);
          float xPosition = current_position[X_AXIS] + 10;
          plan_buffer_line(xPosition, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();

          // we have to let the planner know where we are right now as it is not where we said to go.
          current_position[X_AXIS] = st_get_position_mm(X_AXIS);
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);

          if (!didHitEndstops()) {
            SERIAL_ERROR_START;
            SERIAL_ERRORLNPGM(" xypos failed - no limit hit");
          }

          // Output the trigger position in X in microns.
          SERIAL_PROTOCOLPGM("xypos X: ");
          SERIAL_PROTOCOLLN(float(current_position[X_AXIS]*1000));
          enable_endstops(false);
          plan_buffer_line(current_position[X_AXIS]-2.0, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          enable_endstops(true);
      }
      break;

    case 21: // G21: XYPositioner X2 - Move in -X until switch triggered
      {
          enable_endstops(true);
          // move in -X until a switch is triggered
          feedrate = homing_feedrate[X_AXIS]/(2);
          float xPosition = current_position[X_AXIS] - 10;
          plan_buffer_line(xPosition, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();

          // we have to let the planner know where we are right now as it is not where we said to go.
          current_position[X_AXIS] = st_get_position_mm(X_AXIS);
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);

          if (!didHitEndstops()) {
            SERIAL_ERROR_START;
            SERIAL_ERRORLNPGM(" xypos failed - no limit hit");
          }

          // Output the trigger position in X in microns.
          SERIAL_PROTOCOLPGM("xypos X: ");
          SERIAL_PROTOCOLLN(float(current_position[X_AXIS]*1000));
          enable_endstops(false);
          plan_buffer_line(current_position[X_AXIS]+2.0, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          enable_endstops(true);
      }
      break;

    case 24: //Test the zAxis - move to impossible position, and report where limit switch triggered.
      {
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
      break;

    case 25: //Test the xAxis - move to impossible position, and report where limit switch triggered.
      {
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
      break;

      case 26: //Test the yAxis - move to impossible position, and report where limit switch triggered.
      {
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
      break;

    case 27: //Test the zAxis - move to impossible position, and report where limit switch triggered.
      {

          feedrate = homing_feedrate[Z_AXIS]/(6);
          // move down until you find the bed
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
      break;
    case 30: // G30 Single Z Probe
        {
            st_synchronize();
            setup_for_endstop_move(); // VOLTERA EDITED

            feedrate = homing_feedrate[Z_AXIS];

            run_z_probe();

            if (!didHitEndstops()) {
              SERIAL_ERROR_START;
              SERIAL_ERRORLNPGM(" probe failed - no limit hit");
            }

            SERIAL_PROTOCOLPGM(MSG_BED);
            SERIAL_PROTOCOLPGM(" Z: ");
            SERIAL_PROTOCOL(1000*current_position[Z_AXIS]);
            SERIAL_PROTOCOLPGM("\n");

            clean_up_after_endstop_move();
        }
      break;

      case 31: //G31 Reports the Probe Offset
      {
         feedrate = homing_feedrate[Z_AXIS];
          // move down until you find the bed
          float zPosition = -10;
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();

          if (!didHitEndstops()) {
              SERIAL_ERROR_START;
              SERIAL_ERRORLNPGM(" probe bottom pad failed - no limit hit");
          }

          // We hit the bed, tell the planner know where we are right now as it is not where we said to go.
          current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);

          // Copy this value,  this is the height of our GOLD pad (p_bot)
          float p_bot_height = current_position[Z_AXIS] * 1000;

          // Override the p_bot, we will continue to go down until the probe triggers
          override_p_bot = true;
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();

          // Stop overriding it, we are moving up now. Tell the planner where we are right now.
          override_p_bot = false;
          current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS]);

          float z_probe_offset = p_bot_height - current_position[Z_AXIS]*1000;

          SERIAL_PROTOCOLPGM("Probe Offset: ");
          SERIAL_PROTOCOL(z_probe_offset);
          SERIAL_PROTOCOLPGM("\n");
      }
      break;
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
           if(i == E_AXIS) {
             current_position[i] = code_value();
             plan_set_e_position(current_position[E_AXIS]);
           }
           else {
             current_position[i] = code_value()+add_homeing[i];
             plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           }
        }
      }
      break;
    }
    previous_millis_active_cmd = millis();
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {

    case 17:
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
      break;

    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      autotempShutdown();
      }
      break;
    case 104: // M104
      if(setTargetedHotend(104)){
        break;
      }
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
      setWatch();
      break;
    case 140: // M140 set bed temp
      previous_millis_active_cmd = millis();
      if (code_seen('S')) setTargetBed(code_value());
      break;
    case 105 : // M105
      if(setTargetedHotend(105)){
        break;
        }
     // #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_OK " ");
        SERIAL_PROTOCOL(gcode_N);
        SERIAL_PROTOCOLPGM(" T:");
        SERIAL_PROTOCOL_F(0.0,1);
        //SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(0.0,1 );
        //SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
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
      return;
      break;
    case 190: // M190 - Wait for bed heater to reach target.
    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        if (code_seen('S')) {
          setTargetBed(code_value());
          CooldownNoWait = true;
        } else if (code_seen('R')) {
          setTargetBed(code_value());
          CooldownNoWait = false;
        }
        codenum = millis();

        target_direction = isHeatingBed(); // true if heating, false if cooling
        pending_temp_change = true;

        while ( target_direction ? (isHeatingBed()) : (isCoolingBed()&&(CooldownNoWait==false)) )
        {
          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            float tt=degHotend(active_extruder);
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL(tt);
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL((int)active_extruder);
            SERIAL_PROTOCOLPGM(" B:");
            SERIAL_PROTOCOL_F(degBed(),1);
            SERIAL_PROTOCOLLN("");
            codenum = millis();
          }
          manage_heater();
          manage_inactivity();
        }
        pending_temp_change = false;
        previous_millis_active_cmd = millis();
    #endif
        break;

      #ifdef VOLTERA
        case 93: // M93 Manually control LEDs
          // Syntax is M93 R:nnn V:nnn B:nnn (0 <= nnn <= 255)
          // V because vert (using 'G' derails the parser, unsurprisingly)
          // (Call with no arguments to release LEDs)

          if (code_seen('R')) analogWrite(LED_RED_PIN, constrain(code_value(), 0, 255));
          if (code_seen('V')) analogWrite(LED_GREEN_PIN, constrain(code_value(), 0, 255));
          if (code_seen('B')) analogWrite(LED_BLUE_PIN, constrain(code_value(), 0, 255));

          glow_led_override = code_seen('R') || code_seen('V') || code_seen('B');
        break;

      #endif

    #if defined(FAN_PIN) && FAN_PIN > -1
      case 106: //M106 Fan On
        if (code_seen('S')){
           fanSpeed=constrain(code_value(),0,255);
        }
        else {
          fanSpeed=255;
        }
        break;
      case 107: //M107 Fan Off
        fanSpeed = 0;
        break;
    #endif //FAN_PIN

      case 81: // M81 - Turn off Power Supply
        disable_heater();
        st_synchronize();
        finishAndDisableSteppers();
        fanSpeed = 0;
        delay(1000); // Wait a little before to switch off
	  break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
          st_synchronize();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) {disable_z();}
          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
            }
          #endif
        }
      }
      break;
    case 85: // M85
      code_seen('S');
      max_no_serial_no_movement_time = code_value() * 1000;
      break;
    case 92: // M92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;

    #ifdef VOLTERA
      case 112:
        quickStop();
        // We can optionally reset the planner to the stepper counts in some axes
        if (code_seen(axis_codes[X_AXIS])) current_position[X_AXIS] = st_get_position_mm(X_AXIS);
        if (code_seen(axis_codes[Y_AXIS])) current_position[Y_AXIS] = st_get_position_mm(Y_AXIS);
        if (code_seen(axis_codes[Z_AXIS])) current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
        if (code_seen(axis_codes[E_AXIS])) current_position[E_AXIS] = st_get_position_mm(E_AXIS);
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        break;

      case 102:
        override_p_bot = true;
        break;
      case 103:
        override_p_bot = false;
        break;

    #endif

    case 115: // M115
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      break;

    case 114: // M114
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
      SERIAL_PROTOCOL(axis_homed_state[X_AXIS]);
      SERIAL_PROTOCOL(',');
      SERIAL_PROTOCOL(axis_homed_state[Y_AXIS]);
      SERIAL_PROTOCOL(',');
      SERIAL_PROTOCOL(axis_homed_state[Z_AXIS]);

      SERIAL_PROTOCOLLN("");
      break;
    case 120: // M120 - Added by VOLTERA
      st_synchronize();
      enable_endstops(false) ;
      break;
    case 121: // M121 -  Added by VOLTERA
      st_synchronize();
      enable_endstops(true) ;
      break;
    case 122: //M122 - We let the planner know where we are. -  Added by VOLTERA
        {
          st_synchronize();
          // If no axes are specified, we reset the Z axis (for compat with the old software)
          // Otherwise, we reset the specified axes
          bool z_default = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
          if (code_seen(axis_codes[X_AXIS])) current_position[X_AXIS] = st_get_position_mm(X_AXIS);
          if (code_seen(axis_codes[Y_AXIS])) current_position[Y_AXIS] = st_get_position_mm(Y_AXIS);
          if (code_seen(axis_codes[Z_AXIS]) || z_default) current_position[Z_AXIS]  = st_get_position_mm(Z_AXIS);
          if (code_seen(axis_codes[E_AXIS])) current_position[E_AXIS] = st_get_position_mm(E_AXIS);
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        }
        break;
    case 123: // M123 - set software endstop(s) to current position - specify axis/es and T for top (max) and B for bottom (min). These are reset upon homing
      if (code_seen(axis_codes[X_AXIS])) (code_seen('T') ? max_pos : min_pos)[X_AXIS] = current_position[X_AXIS];
      if (code_seen(axis_codes[Y_AXIS])) (code_seen('T') ? max_pos : min_pos)[Y_AXIS] = current_position[Y_AXIS];
      if (code_seen(axis_codes[Z_AXIS])) (code_seen('T') ? max_pos : min_pos)[Z_AXIS] = current_position[Z_AXIS];

      break;

    case 124: // M124 configure forced-homing behaviour. H0/1 to disable/enable, A0/1 to home all axis immediately, not just those required for the movement
      if (code_seen('H')) ensure_homed_enable = code_value_long() != 0;
      if (code_seen('A')) ensure_homed_preemptive_all_axis = code_value_long() != 0;
      break;

    case 125:
    {
      // Reports the current state of the Probe
      #if defined(P_TOP_STATE_PIN) && P_TOP_STATE_PIN > -1
      float probeVoltage = analogRead(P_TOP_STATE_PIN)/1024.0*5.0;
      SERIAL_PROTOCOLPGM("Probe: ");

      if (probeVoltage < 1.0)
          SERIAL_PROTOCOLLNPGM("TRIGGERED");
      else if(probeVoltage >= 1.0 && probeVoltage <= 4.0)
          SERIAL_PROTOCOLLNPGM("ON");
      else
          SERIAL_PROTOCOLLNPGM("OFF");
      #endif
      break;
    }

    case 119: // M119
    SERIAL_PROTOCOLLN(MSG_M119_REPORT);
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MIN);
        SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MAX);
        SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MIN);
        SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
        SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MIN);
        SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
        SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(P_TOP_PIN) && P_TOP_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_P_TOP);
        SERIAL_PROTOCOLLN(((READ(P_TOP_PIN)^P_TOP_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
        #if defined(P_BOT_PIN) && P_BOT_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_P_BOT);
        SERIAL_PROTOCOLLN(((READ(P_BOT_PIN)^P_BOT_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(XY_MIN_X_PIN) && XY_MIN_X_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_XY_MIN_X);
        SERIAL_PROTOCOLLN(((READ(XY_MIN_X_PIN)^XY_MIN_X_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(XY_MAX_X_PIN) && XY_MAX_X_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_XY_MAX_X);
        SERIAL_PROTOCOLLN(((READ(XY_MAX_X_PIN)^XY_MAX_X_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(XY_MIN_Y_PIN) && XY_MIN_Y_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_XY_MIN_Y);
        SERIAL_PROTOCOLLN(((READ(XY_MIN_Y_PIN)^XY_MIN_Y_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(XY_MAX_Y_PIN) && XY_MAX_Y_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_XY_MAX_Y);
        SERIAL_PROTOCOLLN(((READ(XY_MAX_Y_PIN)^XY_MAX_Y_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      break;
      //TODO: update for all axis, use for loop

    case 200: // M200 D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
      {
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
          break;
        }
        tmp_extruder = active_extruder;
        if(code_seen('T')) {
          tmp_extruder = code_value();
          if(tmp_extruder >= EXTRUDERS) {
            SERIAL_ECHO_START;
            SERIAL_ECHO(MSG_M200_INVALID_EXTRUDER);
          }
          SERIAL_ECHOLN(tmp_extruder);
          break;
        }
        volumetric_multiplier[tmp_extruder] = 1 / area;
      }
      break;
    case 201: // M201
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
      }
      break;
    #endif
    case 203: // M203 max feedrate mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 acclereration S normal moves T filmanent only moves
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 additional homeing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
      }
      break;
    #ifdef DELTA
	case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
		if(code_seen('L')) {
			delta_diagonal_rod= code_value();
		}
		if(code_seen('R')) {
			delta_radius= code_value();
		}
		if(code_seen('S')) {
			delta_segments_per_second= code_value();
		}

		recalc_delta_settings(delta_radius, delta_diagonal_rod);
		break;
    case 666: // M666 set delta endstop adjustemnt
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) endstop_adj[i] = code_value();
      }
      break;
    #endif

    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        int tmp_code = code_value();
        if (code_seen('T'))
        {
          if(setTargetedHotend(221)){
            break;
          }
          extruder_multiply[tmp_extruder] = tmp_code;
        }
        else
        {
          extrudemultiply = tmp_code ;
        }
      }
    }
    break;

    #ifdef PIDTEMP
    case 301: // M301
      {
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = scalePID_i(code_value());
        if(code_seen('D')) Kd = scalePID_d(code_value());

        #ifdef PID_ADD_EXTRUSION_RATE
        if(code_seen('C')) Kc = code_value();
        #endif

        updatePID();
        ACK_CMD
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(Kp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(Ki));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(Kd));
        #ifdef PID_ADD_EXTRUSION_RATE
        SERIAL_PROTOCOL(" c:");
        //Kc does not have scaling applied above, or in resetting defaults
        SERIAL_PROTOCOL(Kc);
        #endif
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    #ifdef PIDTEMPBED
    case 304: // M304
      {
        if(code_seen('P')) bedKp = code_value();
        if(code_seen('I')) bedKi = scalePID_i(code_value());
        if(code_seen('D')) bedKd = scalePID_d(code_value());

        updatePID();
        ACK_CMD
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(bedKp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(bedKi));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(bedKd));
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP

    #ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302: // allow cold extrudes, or set the minimum extrude temperature
    {
	  float temp = .0;
	  if (code_seen('S')) temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
	#endif
    case 303: // M303 PID autotune
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;
    case 400: // M400 finish all moves
    {
      st_synchronize();
    }
    break;
    case 500: // M500 Store settings in EEPROM
    {
        Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
        Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;

    case 504: //M504 - Store the Serial Number.
    {
      if(code_seen('S')) memcpy(product_serial_number, &cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], sizeof(product_serial_number));

      //Terminate the string.
      product_serial_number[10] = '\0';
      Config_StoreCalibration();
    }
    break;

    case 505:
    {
           SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM("Calibration Offsets:");
           SERIAL_ECHO_START;
          SERIAL_ECHOPAIR("X:", min_z_x_pos);
           SERIAL_ECHOPAIR(" Y:", min_z_y_pos);
           SERIAL_ECHOPAIR(" I:", xypos_x_pos);
           SERIAL_ECHOPAIR(" J:", xypos_y_pos);
           SERIAL_ECHOPAIR(" P:", z_probe_offset);
           SERIAL_ECHOLN("");


    }
    break;
    case 508: // M505 Store No. Min X, Min Y, XY Positioner locations.
    {
      if(code_seen('X')) min_z_x_pos = code_value();
      if(code_seen('Y')) min_z_y_pos = code_value();
      if(code_seen('I')) xypos_x_pos = code_value();
      if(code_seen('J')) xypos_y_pos = code_value();
      Config_StoreCalibration();
    }
    break;

    case 506: // Store the axis scaling and axis skew.
    {
      if(code_seen('X')) calib_x_scale = code_value();
      if(code_seen('Y')) calib_y_scale = code_value();
      if(code_seen('A')){// Read the Axis Skew.
        float theta_rad = code_value(); // Save it, as radians save to EEPROM
        calib_cos_theta = cos(theta_rad);
        calib_tan_theta = tan(theta_rad);
      }
      Config_StoreCalibration();
    }
      break;
    //Print Offsets, Calibration, and Serial Number
    case 507:
    {
      Config_PrintCalibration();
    }
    break;
    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    case 540:
    {
        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif

      case 906: // M906 Get all digital potentiometer values.
      {

        #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
          SERIAL_PROTOCOLLN("Stepper Driver Currents (Max: 255)");
          SERIAL_PROTOCOLPGM("X:");
          SERIAL_PROTOCOL((int)digiPotGetCurrent(X_AXIS));
          SERIAL_PROTOCOLPGM("  Y:");
          SERIAL_PROTOCOL((int)digiPotGetCurrent(Y_AXIS));
          SERIAL_PROTOCOLPGM("  Z:");
          SERIAL_PROTOCOL((int)digiPotGetCurrent(Z_AXIS));
          SERIAL_PROTOCOLPGM("  E:");
          SERIAL_PROTOCOLLN((int)digiPotGetCurrent(E_AXIS));
        #endif
      }
      break;

    case 907: // M907 Set digital trimpot motor current using axis codes.
    {

      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digiPotSetCurrent(i,code_value());
      #endif
    }
    break;
    case 908: // M908 Control digital trimpot directly.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digiPotWrite(channel, current);
      #endif
    }
    break;
    }
  }

  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
  }

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  ACK_CMD
}

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
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

void clamp_to_software_endstops(float target[3])
{
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

#ifdef DELTA
void recalc_delta_settings(float radius, float diagonal_rod)
{
	 delta_tower1_x= -SIN_60*radius; // front left tower
	 delta_tower1_y= -COS_60*radius;
	 delta_tower2_x=  SIN_60*radius; // front right tower
	 delta_tower2_y= -COS_60*radius;
	 delta_tower3_x= 0.0;                  // back middle tower
	 delta_tower3_y= radius;
	 delta_diagonal_rod_2= sq(diagonal_rod);
}

void calculate_delta(float cartesian[3])
{
  delta[X_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower1_x-cartesian[X_AXIS])
                       - sq(delta_tower1_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  delta[Y_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower2_x-cartesian[X_AXIS])
                       - sq(delta_tower2_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  delta[Z_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower3_x-cartesian[X_AXIS])
                       - sq(delta_tower3_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  /*
  SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

  SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
  */
}
#endif

void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_active_cmd = millis();

  if (ensure_homed_enable) {
    ensure_homed(current_position[X_AXIS] != destination[X_AXIS], current_position[Y_AXIS] != destination[Y_AXIS], current_position[Z_AXIS] != destination[Z_AXIS]);
  }

  if (!ensure_requested_homing()) return;

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

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_active_cmd = millis();
}


#ifdef TEMP_STAT_LEDS
static bool blue_led = false;
static bool red_led = false;
static uint32_t stat_update = 0;

void handle_status_leds(void) {
  float max_temp = 0.0;
  if(millis() > stat_update) {
    stat_update += 500; // Update every 0.5s
    for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
       max_temp = max(max_temp, degHotend(cur_extruder));
       max_temp = max(max_temp, degTargetHotend(cur_extruder));
    }
    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
      max_temp = max(max_temp, degTargetBed());
      max_temp = max(max_temp, degBed());
    #endif
    if((max_temp > 55.0) && (red_led == false)) {
      digitalWrite(STAT_LED_RED, 1);
      digitalWrite(STAT_LED_BLUE, 0);
      red_led = true;
      blue_led = false;
    }
    if((max_temp < 54.0) && (blue_led == false)) {
      digitalWrite(STAT_LED_RED, 0);
      digitalWrite(STAT_LED_BLUE, 1);
      red_led = false;
      blue_led = true;
    }
  }
}
#endif

void manage_inactivity()
{
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_active_cmd) >  stepper_inactive_time ){
      if(blocks_queued() == false){
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
      }
    }
  }

  // Power down everything if serial traffic has stopped in addition to a lack of movement
  if (!stepper_inactive_time || !previous_millis_active_cmd || (millis() - previous_millis_active_cmd) >  stepper_inactive_time) {
    if (millis() - previous_millis_serial_rx > max_no_serial_no_movement_time && previous_millis_serial_rx && max_no_serial_no_movement_time) {
      disable_x();
      disable_y();
      disable_z();
      disable_e0();
      disable_heater();
    }
  }


  #ifdef VOLTERA
    handle_glow_leds();
  #endif

  #ifdef TEMP_STAT_LEDS
      handle_status_leds();
  #endif
  check_axes_activity();
}

void checkBufferEmpty() {
  static uint8_t buffer_fill = 0;
  uint8_t new_buffer_fill = movesplanned();
  if (buffer_fill && !new_buffer_fill) {
    SERIAL_PROTOCOLLNPGM("empty");
  }
  buffer_fill = new_buffer_fill;
}

#ifdef VOLTERA
void handle_glow_leds(){
  #define GLOW_LED_COUNT 3
  #define TEMP_PACE_CURVE max((-5 * bedTemp) / 42 + 36, 5)
  static const char glow_led_pins[GLOW_LED_COUNT] = {LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN};
  static const unsigned char sin_lookup[256] = {0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124,128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0};
  static unsigned char glow_led_states[GLOW_LED_COUNT]; // These are prescale values - set to the max you want the LED to hit during the cycle
  static unsigned char glow_led_states_hold[GLOW_LED_COUNT];
  static unsigned short glow_led_counter;
  static unsigned long glow_led_last_tick;
  static unsigned short glow_led_pace; // ms/step
  if (glow_led_override){
    // Set this to 0 so we start from 0 when glow_led_override is unset
    glow_led_counter = 0;
    return;
  }

  /*
  (in order of precedence)
  White                     - No Connection
  Idle                      - Green
  Receiving motion commands - Purple
  Bed temp falling (M190)   - Blue
  Bed temp rising (M190)    - Orange
  Bed temp >= 50degC        - Red

  Blue/orange are also triggered when the target temperature or bed temperature are otherwise notable
  Short of having the desktop software send explicit commands to say "cooling begin" or "cooling end," this is OK
  */
  bool quick_change = false;
  bool ramp_down_now = false;

  float bedTemp = degBed();
  float targetTemp = degTargetBed();
  if(bedTemp > 50.0){
    glow_led_states[0] = 255;
    glow_led_states[1] = 0;
    glow_led_states[2] = 0;
    glow_led_pace = TEMP_PACE_CURVE;
  } else if (pending_temp_change || targetTemp > 0 || bedTemp > 40) {
    if (isCoolingBed()) {
      glow_led_states[0] = 0;
      glow_led_states[1] = 0;
      glow_led_states[2] = 255;
    } else {
      glow_led_states[0] = 255;
      glow_led_states[1] = 40;
      glow_led_states[2] = 0;
    }
    glow_led_pace = TEMP_PACE_CURVE;
  } else if (millis() - previous_millis_active_cmd < stepper_inactive_time && previous_millis_active_cmd !=0){
    glow_led_states[0] = 255;
    glow_led_states[1] = 0;
    glow_led_states[2] = 255;
    glow_led_pace = 30;
    quick_change = true;
  } else if (glow_force_green || (millis() - previous_millis_serial_rx < 1000 && previous_millis_serial_rx != 0)) {
    glow_led_states[0] = 0;
    glow_led_states[1] = 255;
    glow_led_states[2] = 0;
    glow_led_pace = 30;
    quick_change = true;
  } else {
    glow_led_states[0] = 255;
    glow_led_states[1] = 255;
    glow_led_states[2] = 255;
    glow_led_pace = 30;
  }

  if (quick_change && memcmp(glow_led_states, glow_led_states_hold, sizeof(glow_led_states))) {
    // Go fast till we hit 0 (and glow_led_states is copied into glow_led_states_hold)
    glow_led_pace /= 10;
    ramp_down_now = true;
  }

  if ((millis() - glow_led_last_tick) > glow_led_pace) {
    glow_led_last_tick = millis();
    if (glow_led_counter == 0) {
      // To avoid abrupt changes, we wait for zero-crossing before updating the actual state (_hold) from the input
      memcpy(glow_led_states_hold, glow_led_states, sizeof(glow_led_states));
      ramp_down_now = false; // We're at 0, don't go any further backwards...
    }
    // Wrap the counter
    glow_led_counter += ramp_down_now ? (glow_led_counter > 128 ? 1 : -1) : 1;
    if (glow_led_counter >= 256) {
      glow_led_counter = 0;
    }
    // Remap into a sine wave
    // "wow, this circuit printer's indicator LEDs follow a sine wave!" - nobody
    unsigned short glow_led_wrap = sin_lookup[glow_led_counter];
    for (char i = 0; i < GLOW_LED_COUNT; ++i) {
      analogWrite(glow_led_pins[i], (glow_led_wrap * glow_led_states_hold[i]) / 256);
    }
  }
}
#endif

void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();

  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop()
{
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

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

