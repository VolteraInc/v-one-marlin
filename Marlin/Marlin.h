// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#pragma once

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"

#ifndef AT90USB
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
  //Arduino < 1.0.0 does not define this, so we need to do it ourselves
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#ifdef AT90USB
#include "HardwareSerial.h"
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef AT90USB
   #ifdef BTENABLED
         #define MYSERIAL bt
   #else
         #define MYSERIAL Serial
   #endif // BTENABLED
#else
  #define MYSERIAL MSerial
#endif

#define SERIAL_PROTOCOL(x) (MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) (MYSERIAL.print((x),(y)))
#define SERIAL_PROTOCOLPGM(x) (serialprintPGM(PSTR(x)))
#define SERIAL_PROTOCOLLN(x) (MYSERIAL.println(x))
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(PSTR(x)),MYSERIAL.print('\n'))


const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";

#define SERIAL_ERROR_START (serialprintPGM(errormagic))
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START (serialprintPGM(echomagic))
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)
#define SERIAL_ECHO_F(x,y) SERIAL_PROTOCOL_F((x),(y))

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);

// For serial printing from PROGMEM. (Saves loads of SRAM.)
FORCE_INLINE void serialprintPGM(const char* str) {
  while (char ch = pgm_read_byte(str++)) MYSERIAL.write(ch);
}


// Code that blocks/spins/waits should call this so that things like heating
// and motor inactivity properly monitored
void periodic_work();

void setHomedState(int axis, int value);
#define enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); setHomedState(X_AXIS, 0); }

#define enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() { WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); setHomedState(Y_AXIS, 0); }

#define enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); setHomedState(Z_AXIS, 0); }

#define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
#define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void FlushSerialRequestResend();
void ClearToSend();

void kill();

void handle_glow_leds();

void refresh_cmd_timeout(void);

extern const float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent) for all extruders
extern int extruder_multiply[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS] ;

extern float min_pos[3];
extern float max_pos[3];
extern char product_serial_number[15];
extern float min_z_x_pos;
extern float min_z_y_pos;
extern float xypos_x_pos;
extern float xypos_y_pos;

extern const char axis_codes[NUM_AXIS];


extern bool logging_enabled;

// Handling multiple extruders pins
extern uint8_t active_extruder;

#ifdef DIGIPOT_I2C
extern void digipot_i2c_set_current( int channel, float current );
extern void digipot_i2c_init();
#endif

void setStepperInactiveDuration(unsigned long duration);
