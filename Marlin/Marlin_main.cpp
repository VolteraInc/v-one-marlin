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

#include "version.h"
#include "Marlin.h"
#include "macros.h"
#include "planner.h"
#include "stepper.h"
#include "temperature_profile.h"
#include "ConfigurationStore.h"

#include "src/api/api.h"
#include "src/work/work.h"
#include "src/vone/VOne.h"

// Define missing function: placement-new
void * operator new (size_t, void * ptr) { return ptr; }

//===========================================================================
//=============================public variables=============================
//===========================================================================
VOne* vone;

bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float volumetric_multiplier[EXTRUDERS] = {1.0};
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

uint8_t active_extruder = 0;

float min_z_x_pos;
float min_z_y_pos;
float xypos_x_pos;
float xypos_y_pos;
char product_serial_number[15];
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

bool logging_enabled = false;

//===========================================================================
//=============================Routines======================================
//===========================================================================

extern "C" {
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

void setup() {
  MYSERIAL.begin(BAUDRATE);

  // sometimes there are noise characters in the first message,
  // so we make the first message a bit more distinct by including dashes
  SERIAL_PROTOCOLLNPGM("--start--");

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu) {
    SERIAL_ECHO_START;
    if(mcu & 1) SERIAL_ECHOLNPGM("PowerUp");
    if(mcu & 2) SERIAL_ECHOLNPGM(" External Reset");
    if(mcu & 4) SERIAL_ECHOLNPGM(" Brown out Reset");
    if(mcu & 8) SERIAL_ECHOLNPGM(" Watchdog Reset");
    if(mcu & 32) SERIAL_ECHOLNPGM(" Software Reset");
  }
  MCUSR = 0;

  SERIAL_ECHO_START;
  SERIAL_PAIR(" Free Memory: ", freeMemory());
  SERIAL_PAIR("  PlannerBufferBytes: ", (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  SERIAL_EOL;

  SERIAL_PROTOCOLPGM("firmwareVersionReport: ");
  SERIAL_PROTOCOLPGM(VERSION_STRING);
  SERIAL_PROTOCOLPGM("\n");

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
  sendToolStatusUpdate();



  plan_init();  // Initialize planner
  st_init();    // Initialize stepper
  manufacturing_init();

  // Configure calling frequency of TIMER0_COMPB_vect
  // NOTE: Timer 0 is used by millis() so don't change the prescaler
  OCR0B = 128;
  SBI(TIMSK0, OCIE0B);

  sei();

  SERIAL_PROTOCOLLNPGM("ready");
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

  // This work is excluded from periodic_work() becuase we
  // don't want to do these in the middle of processing a command
  checkForEndstopHits(); // will detect expected hits as errors
  reportBufferEmpty();   // not important enough to monitor
  periodic_output();     // will generate excessive output
  toolChanges();         // handling a tool change mid command would be needlessly complicated
}

ISR(TIMER0_COMPB_vect) {
  vone->periodicInterruptibleWork();
}
