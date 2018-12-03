#include "trinamicMotors.h"

#include "../../../pins.h"
#include "../../libraries/TMC2130Stepper/src/TMC2130Stepper.h"

const int motorPins[4][4] = {
  {X_ENABLE_PIN, X_DIR_PIN, X_STEP_PIN, X_CS_PIN},
  {Y_ENABLE_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_CS_PIN},
  {Z_ENABLE_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_CS_PIN},
  {E_ENABLE_PIN, E_DIR_PIN, E_STEP_PIN, E_CS_PIN},
};

TMC2130Stepper TMC2130_x = TMC2130Stepper(motorPins[X_AXIS][0], motorPins[X_AXIS][1], motorPins[X_AXIS][2], motorPins[X_AXIS][3]);
TMC2130Stepper TMC2130_y = TMC2130Stepper(motorPins[Y_AXIS][0], motorPins[Y_AXIS][1], motorPins[Y_AXIS][2], motorPins[Y_AXIS][3]);
TMC2130Stepper TMC2130_z = TMC2130Stepper(motorPins[Z_AXIS][0], motorPins[Z_AXIS][1], motorPins[Z_AXIS][2], motorPins[Z_AXIS][3]);
TMC2130Stepper TMC2130_e = TMC2130Stepper(motorPins[E_AXIS][0], motorPins[E_AXIS][1], motorPins[E_AXIS][2], motorPins[E_AXIS][3]);

TMC2130Stepper TMC2130[4] = { TMC2130_x, TMC2130_y, TMC2130_z, TMC2130_e };

void trinamicInit() {

  // 90000 pps -> 186 tStep
  // 100000 pps ->  167 tStep


  const float homing_feedrate[] = HOMING_FEEDRATE;

  /**************  X AXIS ****************/

  TMC2130[X_AXIS].begin(); // Initializes Pins and SPI.
  TMC2130[X_AXIS].off_time(4);
  TMC2130[X_AXIS].blank_time(36);
  TMC2130[X_AXIS].hysteresis_start(1);
  TMC2130[X_AXIS].hysteresis_end(2);

  // Current and Microsteps
  TMC2130[X_AXIS].setCurrent(180, 0.33, 0.7); //Current in milliamps, sense resistor value, holding current fraction.
  TMC2130[X_AXIS].microsteps(16); // Out of the box, we are using 16 microsteps.
  TMC2130[X_AXIS].interpolate(1); // Interpolate the 16 stepping to 256 stepping.
  TMC2130[X_AXIS].hold_delay(8); // How long to wait after standstill is detected before powering down.
  TMC2130[X_AXIS].power_down_delay(128); // Wait about 2 seconds before reducing current.

  // Stealthchop settings
  TMC2130[X_AXIS].stealthChop(1);
  TMC2130[X_AXIS].stealth_autoscale(1);
  TMC2130[X_AXIS].stealth_freq(1);
  TMC2130[X_AXIS].stealth_amplitude(255); //0...255
  TMC2130[X_AXIS].stealth_gradient(4); // 1...15
  TMC2130[X_AXIS].stealth_max_speed(XY_STEALTH_MAX_SPEED); // TPWMTHRS - Upper velocity threshold for stealChop //250

  // Stallguard settings
  TMC2130[X_AXIS].coolstep_min_speed(XY_COOLSTEP_MIN_SPEED); // TCOOLTHRS - Velocity threshold of when to turn on stallguard/coolstep feature.
  TMC2130[X_AXIS].diag1_active_high(1);
  TMC2130[X_AXIS].diag1_stall(1); // Enable DIAG0 active on motor stall
  TMC2130[X_AXIS].sg_stall_value(30); // -64...0....64 - Tunes sensitivity.

  /**************  Y AXIS ****************/

  TMC2130[Y_AXIS].begin(); // Initializes Pins and SPI.
  TMC2130[Y_AXIS].off_time(4);
  TMC2130[Y_AXIS].blank_time(36);
  TMC2130[Y_AXIS].hysteresis_start(1);
  TMC2130[Y_AXIS].hysteresis_end(2);

  // Current and Microsteps
  TMC2130[Y_AXIS].setCurrent(300, 0.33, 0.7); //Current in milliamps, sense resistor value, holding current fraction.
  TMC2130[Y_AXIS].microsteps(16); // Out of the box, we are using 16 microsteps.
  TMC2130[Y_AXIS].interpolate(1); // Interpolate the 16 stepping to 256 stepping.
  TMC2130[Y_AXIS].hold_delay(8); // How long to wait after standstill is detected before powering down.
  TMC2130[Y_AXIS].power_down_delay(128); // Wait about 2 seconds before reducing current.

  // Stealthchop settings
  TMC2130[Y_AXIS].stealthChop(1);
  TMC2130[Y_AXIS].stealth_autoscale(1);
  TMC2130[Y_AXIS].stealth_freq(1);
  TMC2130[Y_AXIS].stealth_amplitude(255); //0...255
  TMC2130[Y_AXIS].stealth_gradient(4); // 1...15
  TMC2130[Y_AXIS].stealth_max_speed(XY_STEALTH_MAX_SPEED); // TPWMTHRS - Upper velocity threshold for stealChop  //250

  // Stallguard settings
  TMC2130[Y_AXIS].coolstep_min_speed(XY_COOLSTEP_MIN_SPEED); // TCOOLTHRS - Velocity threshold of when to turn on stallguard/coolstep feature.
  TMC2130[Y_AXIS].diag1_active_high(1);
  TMC2130[Y_AXIS].diag1_stall(1); // Enable DIAG0 active on motor stall
  TMC2130[Y_AXIS].sg_stall_value(10); // -64...0....64 - Tunes sensitivity

  /**************  Z AXIS ****************/

  TMC2130[Z_AXIS].begin(); // Initializes Pins and SPI.
  TMC2130[Z_AXIS].off_time(4);
  TMC2130[Z_AXIS].blank_time(36);
  TMC2130[Z_AXIS].hysteresis_start(1);
  TMC2130[Z_AXIS].hysteresis_end(2);

  // Current and Microsteps
  TMC2130[Z_AXIS].setCurrent(180, 0.33, 0.7); //Current in milliamps, sense resistor value, holding current fraction.
  TMC2130[Z_AXIS].microsteps(16); // Out of the box, we are using 16 microsteps.
  TMC2130[Z_AXIS].interpolate(1); // Interpolate the 16 stepping to 256 stepping.
  TMC2130[Z_AXIS].hold_delay(8); // How long to wait after standstill is detected before powering down.
  TMC2130[Z_AXIS].power_down_delay(128); // Wait about 2 seconds before reducing current.

  // Stealthchop settings
  TMC2130[Z_AXIS].stealthChop(1);
  TMC2130[Z_AXIS].stealth_autoscale(1);
  TMC2130[Z_AXIS].stealth_freq(1);
  TMC2130[Z_AXIS].stealth_amplitude(255); //0...255
  TMC2130[Z_AXIS].stealth_gradient(4); // 1...15
  TMC2130[Z_AXIS].stealth_max_speed(0); // TPWMTHRS - Upper velocity threshold for stealChop


  /**************  E AXIS ****************/
  // From Trinamic Development board - good speed is: 6000 pps.
  // 6000 pps @ 8 microstepping = ~ 167.
  // 5700 pps @ 8 microstepping = ~ 175.

  TMC2130[E_AXIS].begin(); // Initializes Pins and SPI.
  TMC2130[E_AXIS].off_time(4);
  TMC2130[E_AXIS].blank_time(36);
  TMC2130[E_AXIS].hysteresis_start(1);
  TMC2130[E_AXIS].hysteresis_end(2);

  // Current and Microsteps
  TMC2130[E_AXIS].setCurrent(180, 0.33, 0.7); //Current in milliamps, sense resistor value, holding current fraction.
  TMC2130[E_AXIS].microsteps(16); // Out of the box, we are using 16 microsteps.
  TMC2130[E_AXIS].interpolate(1); // Interpolate the 16 stepping to 256 stepping.
  TMC2130[E_AXIS].hold_delay(8); // How long to wait after standstill is detected before powering down.
  TMC2130[E_AXIS].power_down_delay(128); // Wait about 2 seconds before reducing current.

  //Stealthchop settings
  TMC2130[E_AXIS].stealthChop(1);
  TMC2130[E_AXIS].stealth_autoscale(1);
  TMC2130[E_AXIS].stealth_freq(1);
  TMC2130[E_AXIS].stealth_amplitude(255); //0...255
  TMC2130[E_AXIS].stealth_gradient(4); // 1...15
  TMC2130[E_AXIS].stealth_max_speed(E_STEALTH_MAX_SPEED); // TPWMTHRS - Upper velocity threshold for stealChop

  // Stallguard settings
  //enable_e_max_endstops(true);
  //TMC2130[E_AXIS].coolstep_min_speed(E_COOLSTEP_MIN_SPEED);
  TMC2130[E_AXIS].coolstep_min_speed(0); // TCOOLTHRS - Velocity threshold of when to turn on stallguard/coolstep feature.
  TMC2130[E_AXIS].diag1_active_high(1);
  TMC2130[E_AXIS].diag1_stall(1); // Enable DIAG0 active on motor stall
  TMC2130[E_AXIS].sg_stall_value(0); // -63...0....63 - Tunes sensitivity
}

// SETTERs
void trinamicSetCurrent(int axis, int current) {
  TMC2130[axis].setCurrent(current, 0.33, 0.7);
}

void trinamicSetSG(int axis, int value) {
  TMC2130[axis].sg_stall_value(value);
}

void trinamicSetCoolstepMinSpeed(int axis, int min_speed) {
  TMC2130[axis].coolstep_min_speed(min_speed);
}

void trinamicSetStealthMaxSpeed(int axis, int max_speed) {
  TMC2130[axis].stealth_max_speed(max_speed);
}

// GETTERs
uint32_t trinamicGetTStep(int axis) {
  return TMC2130[axis].microstep_time();
}

int trinamicGetCurrentScaling(int axis) {
  return (int)TMC2130[axis].run_current();
}

int trinamicGetStalled(int axis) {
  return (int)(TMC2130[axis].DRVSTATUS() >> 24 & 0x1);
}

int trinamicGetStallGuard(int axis){
  uint32_t value = TMC2130[axis].DRVSTATUS();
  return (int)(value & 0x03FF);
}

int trinamicGetSG(int axis) {
  return (int) TMC2130[axis].sg_stall_value();
}

uint32_t trinamicGetDRVSTATUS(int axis) {
  return TMC2130[axis].DRVSTATUS();
}

uint8_t trinamicGetGStat(int axis) {
  return TMC2130[axis].GSTAT();
}
