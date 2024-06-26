#pragma once

#include "Configuration.h"

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#ifdef MODEL == 8
  #define X_STEP_PIN         36
  #define X_DIR_PIN          37
  #define X_ENABLE_PIN       41
  #define X_CS_PIN           35
  #define X_MIN_PIN          20
  #define X_LIM_PIN          40

  #define Y_STEP_PIN         24
  #define Y_DIR_PIN          25
  #define Y_ENABLE_PIN       27
  #define Y_CS_PIN           23
  #define Y_MIN_PIN          65
  #define Y_LIM_PIN          26

  #define Z_STEP_PIN        29
  #define Z_DIR_PIN         39
  #define Z_ENABLE_PIN      15
  #define Z_CS_PIN          28
  #define Z_MAX_PIN         19

  #define E_STEP_PIN        31
  #define E_DIR_PIN         32
  #define E_LIM_PIN         33
  #define E_ENABLE_PIN      34
  #define E_CS_PIN          30

  //wip
  #define ADC_PWDN          57
  #define ADC_RST           58
  #define XYZ_START         60
  #define XYZ_DATA_RDY      62
  #define P_BOT_PIN         64
  #define XYZ_CS_PIN        67

  //these define virtual endstops, negative is a useful shorthand here, to-do refactor
  #define XY_MIN_X_PIN      80//-1
  #define XY_MAX_X_PIN      81//-2
  #define XY_MIN_Y_PIN      82//-3
  #define XY_MAX_Y_PIN      83//-4
  #define Z_MIN_PIN         84//-5

  //end of wip

  #define P_TOP_PIN         56
  #define P_TOP_ANALOG_PIN  2

  #define LED_RED_PIN        5
  #define LED_GREEN_PIN      2
  #define LED_BLUE_PIN       4

  #define HEATER_BED_PIN   3   // ANALOG NUMBERING
  #define TEMP_BED_PIN     1   // ANALOG NUMBERING

#elif MODEL == 6
  #define X_STEP_PIN         36
  #define X_DIR_PIN          37
  #define X_ENABLE_PIN       41
  #define X_CS_PIN           35
  #define X_MIN_PIN          20
  #define X_LIM_PIN          40

  #define Y_STEP_PIN         24
  #define Y_DIR_PIN          25
  #define Y_ENABLE_PIN       27
  #define Y_CS_PIN           23
  #define Y_MIN_PIN          65
  #define Y_LIM_PIN          26

  #define Z_STEP_PIN        29
  #define Z_DIR_PIN         39
  #define Z_ENABLE_PIN      15
  #define Z_CS_PIN          28
  #define Z_MIN_PIN         64
  #define Z_MAX_PIN         19

  #define E_STEP_PIN        31
  #define E_DIR_PIN         32
  #define E_LIM_PIN         33
  #define E_ENABLE_PIN      34
  #define E_CS_PIN          30

  #define XY_MIN_X_PIN      62
  #define XY_MAX_X_PIN      60
  #define XY_MIN_Y_PIN      61
  #define XY_MAX_Y_PIN      63

  #define P_BOT_PIN         54

  #define P_TOP_PIN         56
  #define P_TOP_ANALOG_PIN  2

  #define LED_RED_PIN        5
  #define LED_GREEN_PIN      2
  #define LED_BLUE_PIN       4

  #define HEATER_BED_PIN   3   // ANALOG NUMBERING
  #define TEMP_BED_PIN     1   // ANALOG NUMBERING

#else
  // V-Ones before batch 6
  #define X_STEP_PIN         30
  #define X_DIR_PIN          31
  #define X_ENABLE_PIN       15
  #define X_MIN_PIN          34

  #define Y_STEP_PIN         24
  #define Y_DIR_PIN          25
  #define Y_ENABLE_PIN       23
  #define Y_MIN_PIN          22

  #define Z_STEP_PIN        27
  #define Z_DIR_PIN         28
  #define Z_ENABLE_PIN      26
  #define Z_MIN_PIN         69
  #define Z_MAX_PIN         33

  #define E_STEP_PIN        39
  #define E_DIR_PIN         14
  #define E_ENABLE_PIN      29

  #define XY_MIN_X_PIN      64
  #define XY_MAX_X_PIN      60
  #define XY_MIN_Y_PIN      62
  #define XY_MAX_Y_PIN      66

  #define P_BOT_PIN         54

  #define P_TOP_PIN         56
  #define P_TOP_ANALOG_PIN  2

  #define LED_RED_PIN        5
  #define LED_GREEN_PIN      2
  #define LED_BLUE_PIN       4

  #define HEATER_BED_PIN   3   // ANALOG NUMBERING
  #define TEMP_BED_PIN     1   // ANALOG NUMBERING

#endif // Trinamic Drivers

#define DIGIPOTSS_PIN 36
#define DIGIPOT_ADDRESS {0x70,0x60,0x00,0x10} // X Y Z E digipot channels to stepper driver mapping
// Refer to http://ww1.microchip.com/downloads/en/DeviceDoc/22242A.pdf
