/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef stepper_h
#define stepper_h

// Mod by VOLTERA
#define WRITE_Z_STEP(v) WRITE(Z_STEP_PIN, v)
#define NORM_Z_DIR() WRITE(Z_DIR_PIN, !INVERT_Z_DIR)
#define REV_Z_DIR() WRITE(Z_DIR_PIN, INVERT_Z_DIR)

#define WRITE_E_STEP(v) WRITE(E0_STEP_PIN, v)
#define NORM_E_DIR() WRITE(E0_DIR_PIN, !INVERT_E0_DIR)
#define REV_E_DIR() WRITE(E0_DIR_PIN, INVERT_E0_DIR)

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
extern bool abort_on_endstop_hit;
#endif

// Initialize and start the stepper motor subsystem
void st_init();

// Block until all buffered steps are executed
void st_synchronize();

// Set current position in steps
void st_set_position(const long &x, const long &y, const long &z, const long &e);
void st_set_e_position(const long &e);

// Get current position in steps
long st_get_position(uint8_t axis);
float st_get_position_mm(uint8_t axis);
// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
// to notify the subsystem that it is time to go to work.
void st_wake_up();


bool endstop_triggered(int axis);

void checkHitEndstops(); //call from somewhere to create an serial error message with the locations the endstops where hit, in case they were triggered
void clear_endstop(int axis);

void enable_calibration_plate(bool enable);

void checkStepperErrors(); //Print errors detected by the stepper

void finishAndDisableSteppers();

void quickStop();

void digiPotInit();
void digiPotSetCurrent(uint8_t axis, uint8_t current);
uint8_t digiPotGetCurrent(uint8_t axis);
void digiPotWrite(uint8_t address, uint8_t value);
uint8_t digiPotRead(uint8_t address);

#ifdef BABYSTEPPING
  void babystep(const uint8_t axis,const bool direction); // perform a short step with a single stepper motor, outside of any convention
#endif



#endif
