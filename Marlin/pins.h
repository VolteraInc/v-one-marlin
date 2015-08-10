#ifndef PINS_H
#define PINS_H

#define X_MS1_PIN -1
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN -1


/****************************************************************************************
* Arduino Mega pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 77 || MOTHERBOARD == 67 || MOTHERBOARD == 68
#define KNOWN_BOARD 1

//////////////////FIX THIS//////////////
#ifndef __AVR_ATmega1280__
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
#endif


// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0


#if MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 77 || MOTHERBOARD == 67 || MOTHERBOARD == 68

  #define LARGE_FLASH true
  #define Y2_STEP_PIN        -1
  #define Y2_DIR_PIN         -1
  #define Y2_ENABLE_PIN      -1

  #if VOLTERA_PIN_VERSION == 0
    #error 'Wrong Pin Version'

    #define X_STEP_PIN         38//42
    #define X_DIR_PIN          21
    #define X_ENABLE_PIN       43
    #define X_MIN_PIN          11
    #define X_MAX_PIN          -1

    #define Y_STEP_PIN         40
    #define Y_DIR_PIN          41
    #define Y_ENABLE_PIN       37
    #define Y_MIN_PIN          35
    #define Y_MAX_PIN          -1

    #define Z_STEP_PIN        19
    #define Z_DIR_PIN         18
    #define Z_ENABLE_PIN      20
    #define Z_MIN_PIN         34
    #define Z_MAX_PIN         10

    #define E0_STEP_PIN        33
    #define E0_DIR_PIN         32
    #define E0_ENABLE_PIN      36

    #define P_MIN_PIN          12

    #define LED_RED_PIN            45
    #define LED_GREEN_PIN          46
    #define LED_BLUE_PIN           44

    #define HEATER_BED_PIN   47
    #define TEMP_BED_PIN     15   // ANALOG NUMBERING

  #elif VOLTERA_PIN_VERSION == 1 //Pinout for version 3.

    #define X_STEP_PIN         30
    #define X_DIR_PIN          31
    #define X_ENABLE_PIN       15
    #define X_MIN_PIN          35
    #define X_MAX_PIN          -1

    #define Y_STEP_PIN         24
    #define Y_DIR_PIN          25
    #define Y_ENABLE_PIN       23
    #define Y_MIN_PIN          22
    #define Y_MAX_PIN          -1

    #define Z_STEP_PIN        27
    #define Z_DIR_PIN         28
    #define Z_ENABLE_PIN      26
    #define Z_MIN_PIN         69
    #define Z_MAX_PIN         33

    #define E0_STEP_PIN        39
    #define E0_DIR_PIN         14
    #define E0_ENABLE_PIN      29


    #define XY_MIN_X_PIN      64
    #define XY_MAX_X_PIN      62
    #define XY_MIN_Y_PIN      60
    #define XY_MAX_Y_PIN      66
    #define P_MIN_PIN          34
    #define PROBE_STATUS_PIN   3 // Analog NUMBERING

    #define LED_RED_PIN        5
    #define LED_GREEN_PIN      2
    #define LED_BLUE_PIN       4

    #define HEATER_BED_PIN   3
    #define TEMP_BED_PIN     1   // ANALOG NUMBERING

    #define DIGIPOTSS_PIN 36
    #define DIGIPOT_ADDRESS {0x00,0x03,0x02,0x01} // X Y Z E digipot channels to stepper driver mapping
    // Refer to http://www.intersil.com/content/dam/Intersil/documents/isl2/isl23448.pdf
  #endif

  #define SDPOWER            -1
  #define SDSS               -1
  #define LED_PIN            -1
  #define FAN_PIN            -1 // IO pin. Buffer needed



  #define PS_ON_PIN          -1

  #if defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
    #define KILL_PIN           41
  #else
    #define KILL_PIN           -1
  #endif

  #if MOTHERBOARD == 35
    #define HEATER_0_PIN       8
  #else
    #define HEATER_0_PIN       8   // EXTRUDER 1
  #endif

  #define HEATER_1_PIN       -1    // EXTRUDER 2 (FAN On Sprinter)
  #define HEATER_2_PIN       -1
  #define TEMP_0_PIN         -1   // ANALOG NUMBERING
  #define TEMP_1_PIN         -1   // ANALOG NUMBERING
  #define TEMP_2_PIN         -1   // ANALOG NUMBERING


  #ifdef NUM_SERVOS
    #define SERVO0_PIN         11

    #if NUM_SERVOS > 1
      #define SERVO1_PIN         6
    #endif

    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
    #endif

    #if NUM_SERVOS > 3
      #define SERVO3_PIN         4
    #endif
  #endif

  #if MOTHERBOARD == 68
    #define BEEPER 33
  #endif

  #ifdef TEMP_STAT_LEDS
    #if MOTHERBOARD == 67
      #define STAT_LED_RED       6
      #define STAT_LED_BLUE     11
    #endif
  #endif

#endif // MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 77

// SPI for Max6675 Thermocouple

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define MAX_SCK_PIN          52
  #define MAX_MISO_PIN         50
  #define MAX_MOSI_PIN         51
  #define MAX6675_SS       53
#else
  #define MAX6675_SS       49
#endif

#endif //MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 77



#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
/*#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else*/
  #define _E2_PINS
//#endif


#ifdef DISABLE_MAX_ENDSTOPS
#define X_MAX_PIN          -1
#define Y_MAX_PIN          -1
#define Z_MAX_PIN          -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
#define X_MIN_PIN          -1
#define Y_MIN_PIN          -1
#define Z_MIN_PIN          -1
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN,                  \
                        _E0_PINS _E1_PINS _E2_PINS             \
                        analogInputToDigitalPin(TEMP_0_PIN), analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_2_PIN), analogInputToDigitalPin(TEMP_BED_PIN) }
#endif

