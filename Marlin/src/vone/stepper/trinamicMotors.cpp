#include "trinamicMotors.h"

#include "../../../pins.h"
#include "../../libraries/TMC2130Stepper/src/TMC2130Stepper.h"

#define XY_STEALTH_MAX_SPEED (220) // 300
#define XY_COOLSTEP_MIN_SPEED (186)
#define E_STEALTH_MAX_SPEED (400)
#define E_COOLSTEP_MIN_SPEED (160)

TMC2130Stepper m_x(X_ENABLE_PIN, X_DIR_PIN, X_STEP_PIN, X_CS_PIN);
TMC2130Stepper m_y(Y_ENABLE_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_CS_PIN);
TMC2130Stepper m_z(Z_ENABLE_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_CS_PIN);
TMC2130Stepper m_e(E_ENABLE_PIN, E_DIR_PIN, E_STEP_PIN, E_CS_PIN);

TMC2130Stepper& lookup(AxisEnum axis) {
  switch(axis) {
    case X_AXIS: m_x;
    case Y_AXIS: m_y;
    case Z_AXIS: m_z;
    case E_AXIS: m_e;
  }
};

void trinamicInit() {

  // 90000 pps -> 186 tStep
  // 100000 pps ->  167 tStep

  /**************  X AXIS ****************/

  m_x.begin(); // Initializes Pins and SPI.
  m_x.off_time(4);
  m_x.blank_time(36);
  m_x.hysteresis_start(1);
  m_x.hysteresis_end(2);

  // Current and Microsteps
  m_x.setCurrent(180, 0.33, 0.7); //Current in milliamps, sense resistor value, holding current fraction.
  m_x.microsteps(16); // Out of the box, we are using 16 microsteps.
  m_x.interpolate(1); // Interpolate the 16 stepping to 256 stepping.
  m_x.hold_delay(8); // How long to wait after standstill is detected before powering down.
  m_x.power_down_delay(128); // Wait about 2 seconds before reducing current.

  // Stealthchop settings
  m_x.stealthChop(1);
  m_x.stealth_autoscale(1);
  m_x.stealth_freq(1);
  m_x.stealth_amplitude(255); //0...255
  m_x.stealth_gradient(4); // 1...15
  m_x.stealth_max_speed(XY_STEALTH_MAX_SPEED); // TPWMTHRS - Upper velocity threshold for stealChop //250

  // Stallguard settings
  m_x.coolstep_min_speed(XY_COOLSTEP_MIN_SPEED); // TCOOLTHRS - Velocity threshold of when to turn on stallguard/coolstep feature.
  m_x.diag1_active_high(1);
  m_x.diag1_stall(1); // Enable DIAG0 active on motor stall
  m_x.sg_stall_value(30); // -64...0....64 - Tunes sensitivity.

  /**************  Y AXIS ****************/

  m_y.begin(); // Initializes Pins and SPI.
  m_y.off_time(4);
  m_y.blank_time(36);
  m_y.hysteresis_start(1);
  m_y.hysteresis_end(2);

  // Current and Microsteps
  m_y.setCurrent(300, 0.33, 0.7); //Current in milliamps, sense resistor value, holding current fraction.
  m_y.microsteps(16); // Out of the box, we are using 16 microsteps.
  m_y.interpolate(1); // Interpolate the 16 stepping to 256 stepping.
  m_y.hold_delay(8); // How long to wait after standstill is detected before powering down.
  m_y.power_down_delay(128); // Wait about 2 seconds before reducing current.

  // Stealthchop settings
  m_y.stealthChop(1);
  m_y.stealth_autoscale(1);
  m_y.stealth_freq(1);
  m_y.stealth_amplitude(255); //0...255
  m_y.stealth_gradient(4); // 1...15
  m_y.stealth_max_speed(XY_STEALTH_MAX_SPEED); // TPWMTHRS - Upper velocity threshold for stealChop  //250

  // Stallguard settings
  m_y.coolstep_min_speed(XY_COOLSTEP_MIN_SPEED); // TCOOLTHRS - Velocity threshold of when to turn on stallguard/coolstep feature.
  m_y.diag1_active_high(1);
  m_y.diag1_stall(1); // Enable DIAG0 active on motor stall
  m_y.sg_stall_value(10); // -64...0....64 - Tunes sensitivity

  /**************  Z AXIS ****************/

  m_z.begin(); // Initializes Pins and SPI.
  m_z.off_time(4);
  m_z.blank_time(36);
  m_z.hysteresis_start(1);
  m_z.hysteresis_end(2);

  // Current and Microsteps
  m_z.setCurrent(180, 0.33, 0.7); //Current in milliamps, sense resistor value, holding current fraction.
  m_z.microsteps(16); // Out of the box, we are using 16 microsteps.
  m_z.interpolate(1); // Interpolate the 16 stepping to 256 stepping.
  m_z.hold_delay(8); // How long to wait after standstill is detected before powering down.
  m_z.power_down_delay(128); // Wait about 2 seconds before reducing current.

  // Stealthchop settings
  m_z.stealthChop(1);
  m_z.stealth_autoscale(1);
  m_z.stealth_freq(1);
  m_z.stealth_amplitude(255); //0...255
  m_z.stealth_gradient(4); // 1...15
  m_z.stealth_max_speed(0); // TPWMTHRS - Upper velocity threshold for stealChop


  /**************  E AXIS ****************/
  // From Trinamic Development board - good speed is: 6000 pps.
  // 6000 pps @ 8 microstepping = ~ 167.
  // 5700 pps @ 8 microstepping = ~ 175.

  m_e.begin(); // Initializes Pins and SPI.
  m_e.off_time(4);
  m_e.blank_time(36);
  m_e.hysteresis_start(1);
  m_e.hysteresis_end(2);

  // Current and Microsteps
  m_e.setCurrent(180, 0.33, 0.7); //Current in milliamps, sense resistor value, holding current fraction.
  m_e.microsteps(16); // Out of the box, we are using 16 microsteps.
  m_e.interpolate(1); // Interpolate the 16 stepping to 256 stepping.
  m_e.hold_delay(8); // How long to wait after standstill is detected before powering down.
  m_e.power_down_delay(128); // Wait about 2 seconds before reducing current.

  //Stealthchop settings
  m_e.stealthChop(1);
  m_e.stealth_autoscale(1);
  m_e.stealth_freq(1);
  m_e.stealth_amplitude(255); //0...255
  m_e.stealth_gradient(4); // 1...15
  m_e.stealth_max_speed(E_STEALTH_MAX_SPEED); // TPWMTHRS - Upper velocity threshold for stealChop

  // Stallguard settings
  //enable_e_max_endstops(true);
  //m_e.coolstep_min_speed(E_COOLSTEP_MIN_SPEED);
  m_e.coolstep_min_speed(0); // TCOOLTHRS - Velocity threshold of when to turn on stallguard/coolstep feature.
  m_e.diag1_active_high(1);
  m_e.diag1_stall(1); // Enable DIAG0 active on motor stall
  m_e.sg_stall_value(0); // -63...0....63 - Tunes sensitivity
}

// SETTERs
void trinamicSetCurrent(AxisEnum axis, int current) {
  lookup(axis).setCurrent(current, 0.33, 0.7);
}

void trinamicSetSG(AxisEnum axis, int value) {
  lookup(axis).sg_stall_value(value);
}

void trinamicSetCoolstepMinSpeed(AxisEnum axis, int min_speed) {
  lookup(axis).coolstep_min_speed(min_speed);
}

void trinamicSetStealthMaxSpeed(AxisEnum axis, int max_speed) {
  lookup(axis).stealth_max_speed(max_speed);
}

// GETTERs
uint32_t trinamicGetTStep(AxisEnum axis) {
  return lookup(axis).microstep_time();
}

int trinamicGetCurrentScaling(AxisEnum axis) {
  return (int)lookup(axis).run_current();
}

int trinamicGetStalled(AxisEnum axis) {
  return (int)(lookup(axis).DRVSTATUS() >> 24 & 0x1);
}

int trinamicGetStallGuard(AxisEnum axis){
  uint32_t value = lookup(axis).DRVSTATUS();
  return (int)(value & 0x03FF);
}

int trinamicGetSG(AxisEnum axis) {
  return (int) lookup(axis).sg_stall_value();
}

uint32_t trinamicGetDRVSTATUS(AxisEnum axis) {
  return lookup(axis).DRVSTATUS();
}

uint8_t trinamicGetGStat(AxisEnum axis) {
  return lookup(axis).GSTAT();
}
