// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#pragma once

#include "Configuration.h"
#include "serial.h"
#include "fastio.h"
#include "pins.h"

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


void setStepperInactiveDuration(unsigned long duration);

class VOne;
extern VOne* vone;
