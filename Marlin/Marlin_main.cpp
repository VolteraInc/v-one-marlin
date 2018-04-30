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

#include "version.h"
#include "macros.h"
#include "planner.h"
#include "stepper.h"
#include "temperature_profile.h"
#include "ConfigurationStore.h"

// DEFER: someday the Arduino IDE will support local libraries, until then the
// options are a Makefile or relative includes. I've choosen the latter since
// it make it trivial to setup a development environment (especially on Windows)

#include "src/api/api.h"
#include "src/work/work.h"
#include "src/vone/VOne.h"

#include "src/libraries/MemoryFree/MemoryFree.h"
#include "src/libraries/PlacementNew/PlacementNew.h"

//===========================================================================
//=============================public variables=============================
//===========================================================================
VOne* vone = nullptr;

bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float volumetric_multiplier = 1.0;
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

float min_z_x_pos;
float min_z_y_pos;
float xypos_x_pos;
float xypos_y_pos;
char product_serial_number[15];
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

//===========================================================================

void setup() {
  MYSERIAL.begin(BAUDRATE);

  // sometimes there are noise characters in the first message,
  // so we make the first message a bit more distinct by including dashes
  protocol << F("--start--") << endl;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu) {
    if(mcu & 1) log << F("PowerUp") << endl;
    if(mcu & 2) log << F(" External Reset") << endl;
    if(mcu & 4) log << F(" Brown out Reset") << endl;
    if(mcu & 8) log << F(" Watchdog Reset") << endl;
    if(mcu & 32) log << F(" Software Reset") << endl;
  }
  MCUSR = 0;

  log << F("Free Memory: ") << freeMemory() << endl;
  log << F("PlannerBufferBytes: ") << (int)sizeof(block_t)*BLOCK_BUFFER_SIZE << endl;

  protocol << F("firmwareVersionReport: ") << VERSION_STRING << endl;

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  Config_RetrieveCalibration();

  // Preallocate space of the VOne then use
  // placement new to contruct the object
  static byte voneBuffer[sizeof(VOne)];
  vone = new (voneBuffer) VOne(
    P_TOP_PIN,
    P_TOP_ANALOG_PIN,
    TEMP_BED_PIN,
    HEATER_BED_PIN
  );

  sendHomedStatusUpdate();
  vone->pins.outputEndStopStatus();

  manufacturing_init();

  // Configure calling frequency of TIMER0_COMPB_vect
  // NOTE: Timer 0 is used by millis() so don't change the prescaler
  OCR0B = 128;
  SBI(TIMSK0, OCIE0B);

  sei();

  protocol << F("--ready--") << endl;
}

void periodic_work() {
  manage_heating_profile();
  manage_inactivity();
  glow_leds();
  manufacturing_procedures(); /// <--TODO: should not be here
}

void loop() {
  processSerialCommands();

  periodic_work();

  // The work below is excluded from periodic_work() becuase we
  // don't want to do these in the middle of processing a command

  // Detect unexpected end-stop hits caused by asynchronous commands
  // Note: hits for synchronous commands are detected in processSerialCommands
  checkForEndstopHits();

  reportBufferEmpty();   // not important enough to monitor
  periodic_output();     // will generate excessive output
}

// Stepper uses ISR(TIMER1_COMPA_vect)
// see stepper.cpp for details

ISR(TIMER0_COMPB_vect) {
  vone->frequentInterruptibleWork();
}
