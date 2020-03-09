// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#pragma once

#include "Configuration.h"
#include "serial.h"
#include "fastio.h"
#include "pins.h"
#include "Axis.h"

// Code that blocks/spins/waits should call this so that things like heating
// and motor inactivity properly monitored
void periodic_work();
void safe_delay(unsigned long delayMillis);

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
extern float xypos_z_pos;
extern int z_switch_type;


void setStepperInactiveDuration(unsigned long duration);

class VOne;
extern VOne* vone;
