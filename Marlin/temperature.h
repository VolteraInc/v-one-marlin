/*
  temperature.h - temperature controller
  Part of Marlin

  Copyright (c) 2011 Erik van der Zalm

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

#pragma once

#include "Marlin.h"
#include "planner.h"

// public functions
void tp_init();  //initialize the heating
void manage_heater(); //it is critical that this is called periodically.

// low level conversion routines
// do not use these routines and variables outside of temperature.cpp
extern int target_temperature_bed;
extern float current_temperature_bed;

//high level conversion routines, for use outside of temperature.cpp
//inline so that there is no performance decrease.
//deg=degreeCelsius

FORCE_INLINE float degBed() {
  return current_temperature_bed;
};

FORCE_INLINE float degTargetBed() {
  return target_temperature_bed;
};

FORCE_INLINE void setTargetBed(const float &celsius) {
  target_temperature_bed = celsius;
};

FORCE_INLINE bool isHeatingBed() {
  return target_temperature_bed > current_temperature_bed;
};

FORCE_INLINE bool isCoolingBed() {
  return target_temperature_bed < current_temperature_bed;
};

int getSoftPwmBed();
void disable_heater();

//-----------------------------------------------
// ADC reads
// Note: Way easier to add p_top functions here getter here,
// than to pull out ADC reads into their own module

void manage_adc();

enum PTopModes {
  P_TOP_NORMAL_MODE,
  P_TOP_COMMS_WRITE_MODE,
  P_TOP_COMMS_READ_MODE,
};

// safely change p_top usage modes
void set_p_top_mode(enum PTopModes mode);
bool periodicAnalogReadsEnabled();

// Returns most recent sample of p_top's voltage,
float get_p_top_voltage();

// Returns a newly sampled voltage value,
// blocks if a new value is not ready.
float next_p_top_voltage();
