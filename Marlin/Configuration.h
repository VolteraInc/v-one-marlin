#pragma once

#define VONE_batch_0_TO_5 1
#define VONE_batch_6_to_present 6
#define EXPERIMENTAL 9999

#define MODEL VONE_batch_0_TO_5
// #define MODEL VONE_batch_6_to_present
// #define MODEL EXPERIMENTAL

#if MODEL == VONE_batch_0_TO_5
  #define FIRMARE_VARIANT_SUFFIX "_batch0to5"
  #define checkForFirmwareVariantMismatch(batchNumber) (batchNumber >= 6)

#elif MODEL == VONE_batch_6_to_present
  #define TRINAMIC_MOTORS 1
  #define FIRMARE_VARIANT_SUFFIX "_batch6"
  #define checkForFirmwareVariantMismatch(batchNumber) (batchNumber < 6)

#else
  #define TRINAMIC_MOTORS 1
  #define FIRMARE_VARIANT_SUFFIX "_experimental"
  #define checkForFirmwareVariantMismatch(batchNumber) (false)

#endif

// Default Calibration offsets for the Voltera V-One
#define XYPOS_X_POS                 ( 34.1)  // Value used by Will (our production Tech) for batch 6 units, set in Dec 2019
#define XYPOS_Y_POS                 (  4.5)  // Value used by Will (our production Tech) for batch 6 units, set in Dec 2019
#define OFFSET_FROM_XYPOS_TO_MINZ_X      (-29.60) // Distance between center of Min Z and XY Positioner - Taken from PCB and Solidworks CAD files
#define OFFSET_FROM_XYPOS_TO_MINZ_Y      (2.5)  // Distance between center of Min Z and XY Positioner - Taken from PCB and Solidworks CAD files
#define MIN_Z_X_POS       (XYPOS_X_POS + OFFSET_FROM_XYPOS_TO_MINZ_X)
#define MIN_Z_Y_POS       (XYPOS_Y_POS + OFFSET_FROM_XYPOS_TO_MINZ_Y)
#define CALIB_X_SCALE     (1.0)
#define CALIB_Y_SCALE     (1.0)
#define CALIB_COS_THETA   (1.0)
#define CALIB_TAN_THETA   (0.0)
#define CALIB_X_BACKLASH  (0.0) // Default assumed backlash
#define CALIB_Y_BACKLASH  (0.0)

#define XYPOS_Z_POS       (-4.0)

//Default Serial number for the Voltera V-One
#define PRODUCT_SERIAL    ("V1-00-0000-120")


//===========================================================================
//=============================Serial Settings===========================
//===========================================================================
// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

// This determines the communication speed of the printer
#define BAUDRATE 250000

// Transmission to Host Buffer Size
// To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.
// To buffer a simple "ok" you need 4 bytes.
// For ADVANCED_OK (M105) you need 32 bytes.
// For debug-echo: 128 bytes for the optimal speed.
// Other output doesn't need to be that speedy.
// :[0, 2, 4, 8, 16, 32, 64, 128, 256]
#define TX_BUFFER_SIZE 0

// Host Receive Buffer Size
// Without XON/XOFF flow control (see SERIAL_XON_XOFF below) 32 bytes should be enough.
// To use flow control, set this buffer size to at least 1024 bytes.
// :[0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048]
#define RX_BUFFER_SIZE 128

#if RX_BUFFER_SIZE >= 1024
  // Enable to have the controller send XON/XOFF control characters to
  // the host to signal the RX buffer is becoming full.
  //#define SERIAL_XON_XOFF
#endif

// Enable this option to collect and display the maximum
// RX queue usage after transferring a file to SD.
//#define SERIAL_STATS_MAX_RX_QUEUED

// Enable this option to collect and display the number
// of dropped bytes after a file transfer to SD.
//#define SERIAL_STATS_DROPPED_RX

// Enable an emergency-command parser to intercept certain commands as they
// enter the serial receive buffer, so they cannot be blocked.
// Currently handles M108, M112, M410
// Does not work on boards using AT90USB (USBCON) processors!
//#define EMERGENCY_PARSER

// Buffers for command processing
#define MAX_CMD_SIZE 96
#define BUFSIZE 4


//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// Uncomment the following line to enable CoreXY kinematics
//#define COREXY

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Z_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool P_BOT_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool P_TOP_ENDSTOP_INVERTING = true;

const bool Y_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Z_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.

const bool X_LIM_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool Y_LIM_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool E_LIM_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.


const bool XY_MIN_X_ENDSTOP_INVERTING = true;
const bool XY_MAX_X_ENDSTOP_INVERTING = true;
const bool XY_MIN_Y_ENDSTOP_INVERTING = true;
const bool XY_MAX_Y_ENDSTOP_INVERTING = true;
#define READ_PIN(PIN) (READ(PIN ## _PIN) ^ (PIN ## _ENDSTOP_INVERTING))

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0

#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true
#define INVERT_E_DIR true

// ENDSTOP SETTINGS:

#define min_software_endstops false // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits before homing (Full extent of each axis)
// Note: to measure this value move the head head to the max forward-left corner (with the motors off)
// enable logging, then home the printer. The distance traveled should be logged when the planner's
// position is re-synchronized with the stepper after hitting the limit switch.
#define X_MAX_LENGTH_BEFORE_HOMING 134
#define Y_MAX_LENGTH_BEFORE_HOMING 160
#define Z_MAX_TRAVEL_DOWN_BEFORE_HOMING Z_MAX_POS // so we don't hit the bottom of the carriage
#define Z_MAX_TRAVEL_UP_BEFORE_HOMING (Z_MAX_POS - Z_MIN_POS) // so we can climb out of the xy-positioner

// Distance to retract from X,Y switches when homing
#define HOMING_XY_OFFSET 1

// Extents (min and max reachable positions, after homing)
#define X_MAX_POS 128
#define X_MIN_POS 0
#define Y_MAX_POS 157
#define Y_MIN_POS 0

// This is intentionally 1mm less than the actual axis length (~23mm)
// Otherwise it's possible to drive the z carriage into the bottom of the printhead assembly before we hit the soft endstop
// It has no impact on actual printing, as in that case we zero to the bottom limit switch and never attempt to reach the maximum again
// (if we did, we'd hit the bottom limit switch, since with the tool attached the axis length is reduced significantly)
#define Z_MAX_POS 22

#define Z_MIN_POS -6 // low enough to touch the bottom of the xy-positioner (with the probe)

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)
#define E_MAX_LENGTH (20)
//============================= Bed Auto Leveling ===========================

// default settings

/* Stepping
1   400
1/2 800
1/4 1600
1/8 3200
1/16 6400*/

/* For Gear Ratio
micro/step * 200 step / 16 teeth  * 24 teeth / 1 rev * 1 rev / 0.7 mm pitch
1    428.5714285714286
1/2  857.1428571428572
1/4  1714.2857142857144
1/8  3428.571428571429
1/16 6857.142857142858
*/

//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#ifdef TRINAMIC_MOTORS
  #define HOMING_FEEDRATE {2800, 2800, 200 , 55}  // set the homing speeds (mm/min)
  #define DEFAULT_AXIS_STEPS_PER_UNIT   {100.0, 100.0, 1600.0, 6857.142857142858}

#else
  #define HOMING_FEEDRATE {1200, 1200, 200 , 0} // set the homing speeds (mm/min)
  #define DEFAULT_AXIS_STEPS_PER_UNIT   {100.0, 100.0, 1600.0, 1714.2857142857144}
#endif

// Note: if speed is 100, step rate ends up at 10001, just over
//       the threshold for double-stepping (taking 2 steps within
//       one call to the stepper's isr). This happens becuase
//       compensation alogrithms tweak the destination co-ordinates
//       which can result in a slightly increased travel distance.
//       Dropping the rate to 95 eliminates double-stepping,
//       which I'd rather avoid to reduce complexity
//
// For reference:
//   95mm/s = 5700mm/min = 9.5cm/s
#define DEFAULT_MAX_FEEDRATE          {95.0, 95.0, 200.0/60.0, 140.0/60.0}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {200, 200, 100, 50}    // X, Y, Z, E maximum start speed for accelerated moves
#define DEFAULT_ACCELERATION          1000.0    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  50.0   // X, Y, Z and E max acceleration in mm/s^2 for retracts

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                10.0    // (mm/sec)
#define DEFAULT_ZJERK                 1.0    // (mm/sec)
#define DEFAULT_EJERK                 1.0  // (mm/sec)

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// homing hits the endstop, then retracts by this distance, before it tries to slowly bump again
#define X_HOME_RETRACT_MM 3
#define Y_HOME_RETRACT_MM 3
#define Z_HOME_RETRACT_MM 0.5

#define AXIS_RELATIVE_MODES {false, false, false, false}

#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

//default stepper release if idle, default heater timeout if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME (2*60) // 5 mins
#define DEFAULT_HEATER_DEACTIVE_TIME (60*60) // 60 mins

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
#define DIGIPOT_MOTOR_CURRENT {50,180,60,60} // Values 0-255 (x, y, z, e)


//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 0.050 // 50um, based on cutting a .5mm hole with a .4mm bit, which requires traversal of a 100um circle
#define N_ARC_CORRECTION 25

const unsigned int dropsegments=1; //everything with less than this number of steps will be ignored as move and joined with the next movement


//===========================================================================
//=============================Stepper           ============================
//===========================================================================

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ring-buffering.
#define BLOCK_BUFFER_SIZE 16 // maximize block buffer
