// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#pragma once


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

#define HardwareSerial_h // trick to disable the standard HWserial

#include "Arduino.h"

#include "MarlinSerial.h"
#include "macros.h"

#include "WString.h"

#define MYSERIAL MSerial

#define SERIAL_EOL MYSERIAL.print("\n")
#define SERIAL_PAIR(name,value) do{ serialprintPGM(PSTR(name)); MYSERIAL.print(value); } while(0)

#define SERIAL_PROTOCOL(x) (MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) (MYSERIAL.print((x),(y)))
#define SERIAL_PROTOCOLPGM(x) (serialprintPGM(PSTR(x)))
#define SERIAL_PROTOCOLLN(x)    do{ MYSERIAL.print(x); SERIAL_EOL; }while(0)
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(PSTR(x "\n")))

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

#define SERIAL_ECHOPAIR SERIAL_PAIR
//TODO: replace all SERIAL_ECHOPAIR with SERIAL_PAIR

// For serial printing from PROGMEM. (Saves loads of SRAM.)
FORCE_INLINE void serialprintPGM(const char* str) {
  while (char ch = pgm_read_byte(str++)) MYSERIAL.write(ch);
}

template <typename T>
void serialArray(const T A[], size_t size) {
  for (auto i = 0u; i < size; ++i) {
    if (i != 0) {
      MYSERIAL.print(',');
    }
    MYSERIAL.print(A[i]);
  }
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

extern const float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent) for all extruders
extern float volumetric_multiplier; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
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

void setStepperInactiveDuration(unsigned long duration);

class VOne;
extern VOne* vone;
