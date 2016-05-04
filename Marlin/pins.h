#ifndef PINS_H
#define PINS_H

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

  #if VOLTERA_PIN_VERSION == 1 //Pinout for version 1. (What B1 Shipped with)

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
    #define P_TOP_PIN          34

    #define LED_RED_PIN        5
    #define LED_GREEN_PIN      2
    #define LED_BLUE_PIN       4

    #define HEATER_BED_PIN   3
    #define TEMP_BED_PIN     1   // ANALOG NUMBERING

    #define DIGIPOTSS_PIN 36
    #define DIGIPOT_ADDRESS {0x00,0x03,0x02,0x01} // X Y Z E digipot channels to stepper driver mapping
    // Refer to http://www.intersil.com/content/dam/Intersil/documents/isl2/isl23448.pdf

  #elif VOLTERA_PIN_VERSION == 2 //Pinout for version 3. (What B1 Shipped with)

    #define X_STEP_PIN         30
    #define X_DIR_PIN          31
    #define X_ENABLE_PIN       15
    #define X_MIN_PIN          34
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
    #define XY_MAX_X_PIN      60
    #define XY_MIN_Y_PIN      62
    #define XY_MAX_Y_PIN      66
    #define P_BOT_PIN         54
    #define P_TOP_PIN         56
    #define P_TOP_STATE_PIN   2 // Analog NUMBERING

    //The P_TOP pin is measured in both digital and analog,
    // P_TOP_STATE_MIN is used to get ON, OFF, TRIGGERED status.
    // P_TOP_MIN is used probing

    #define LED_RED_PIN        5
    #define LED_GREEN_PIN      2
    #define LED_BLUE_PIN       4

    #define HEATER_BED_PIN   3
    #define TEMP_BED_PIN     1   // ANALOG NUMBERING

    #define DIGIPOTSS_PIN 36
    #define DIGIPOT_ADDRESS {0x70,0x60,0x00,0x10} // X Y Z E digipot channels to stepper driver mapping
    // Refer to http://ww1.microchip.com/downloads/en/DeviceDoc/22242A.pdf
  #endif


  #define SDPOWER            -1
  #define SDSS               -1
  #define LED_PIN            -1
  #define FAN_PIN            -1 // IO pin. Buffer needed

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

#endif //MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 77



#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

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

#endif

