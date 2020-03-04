#include "Stepper.h"

#include "../endstops/Endstop.h"
#include "../endstops/EndstopMonitor.h"

#include "../../../Marlin.h"
#include "../../../planner.h"
#include "../../../stepper.h"
#include "../../../serial.h"
#include "./digipots.h"
#include "./trinamicMotors.h"

Stepper::Stepper(
  EndstopMonitor& endstopMonitor
) : endstopMonitor(endstopMonitor)
  , maxStepsComplete(F("stepper isr finished first set of steps"), F("us"))
  , maxInterruptsAllowed(F("stepper isr allowing serial interrupts"), F("us"))
  , maxCompletedWithoutTriggers(F("stepper isr completed without triggering"), F("us"))
  , maxCompletedWithoutTriggersTics(F("stepper isr completed without triggering"), F(" tics"), 200)
  , maxStepRate(F("stepper step rate"), F("steps/s"))
  , maxStepTiming(F("stepper step timing"), F(" tics"), 1000000UL)
{
  plan_init();  // Initialize planner

  #if TRINAMIC_MOTORS
    // Initialize Trinamic Driver
    // Note: trinamicInit initalizes pins too
    trinamicInit();

  #else
    // Initialize Digipot Motor Current
    digiPotInit();

    //Initialize Dir Pins
    SET_OUTPUT(X_DIR_PIN);
    SET_OUTPUT(Y_DIR_PIN);
    SET_OUTPUT(Z_DIR_PIN);
    SET_OUTPUT(E_DIR_PIN);

    // Initialize Enable Pins - steppers default to disabled.
    SET_OUTPUT(X_ENABLE_PIN);
    SET_OUTPUT(Y_ENABLE_PIN);
    SET_OUTPUT(Z_ENABLE_PIN);
    SET_OUTPUT(E_ENABLE_PIN);
  #endif

  // Initialize Step Pins
  SET_OUTPUT(X_STEP_PIN);
  WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
  disable_x();

  SET_OUTPUT(Y_STEP_PIN);
  WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
  disable_y();

  SET_OUTPUT(Z_STEP_PIN);
  WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
  disable_z();

  SET_OUTPUT(E_STEP_PIN);
  WRITE(E_STEP_PIN, INVERT_E_STEP_PIN);
  disable_e();

  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11);
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0);
  TCCR1A &= ~(3<<COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
}

void Stepper::start() {
  st_start();
}

void Stepper::stop(const __FlashStringHelper* reason) {
  ScopedInterruptDisable sid;
  m_stopReason = reason;
  quickStop();
}

void Stepper::resume() {
  ScopedInterruptDisable sid;
  m_stopReason = nullptr;
}

const __FlashStringHelper* Stepper::stopReason() const {
  ScopedInterruptDisable sid;
  return m_stopReason;
}

bool Stepper::stopped() const {
  return stopReason() != nullptr;
}

void Stepper::finishPendingMoves() const {
  st_synchronize();
}

void Stepper::enableSkewAdjustment(bool enabled) {
  plan_enable_skew_adjustment(enabled);
}

int Stepper::overrideCurrentPosition(float x, float y, float z, float e) {
  // DEFER: Stepper (or planner) should own current_position
  //        everything else should access it through Stepper
  //        (consider renaming to Stepper ot planner.destination)
  current_position[X_AXIS] = x;
  current_position[Y_AXIS] = y;
  current_position[Z_AXIS] = z;
  current_position[E_AXIS] = e;
  plan_set_position(x, y, z, e);
  return 0;
}

int Stepper::overrideCurrentPosition(float position[4]) {
  return overrideCurrentPosition(
    position[X_AXIS],
    position[Y_AXIS],
    position[Z_AXIS],
    position[E_AXIS]
  );
}

int Stepper::overrideCurrentPosition(AxisEnum axis, float value) {
  if (axis == E_AXIS) {
    current_position[E_AXIS] = value;
    plan_set_e_position(value);
    return 0;
  }

  current_position[axis] = value;
  return overrideCurrentPosition(current_position);
}


int Stepper::resyncWithStepCount(AxisEnum axis) {
  return overrideCurrentPosition(axis, st_get_position_mm(axis));
}

int Stepper::resyncWithStepCount(bool x, bool y, bool z, bool e) {
  if (x) { current_position[X_AXIS] = st_get_position_mm(X_AXIS); }
  if (y) { current_position[Y_AXIS] = st_get_position_mm(Y_AXIS); }
  if (z) { current_position[Z_AXIS] = st_get_position_mm(Z_AXIS); }
  if (e) { current_position[E_AXIS] = st_get_position_mm(E_AXIS); }
  return overrideCurrentPosition(current_position);
}

int Stepper::add(float x, float y, float z, float e, float f) {

  // Check if we are stopped
  const auto rejectReason = stopReason();
  if (rejectReason) {
    logError
      << F("Unable to perform movement, ")
      << rejectReason
      << endl;
    return -1;
  }

  plan_buffer_line(x, y, z, e, f / 60);

  // Check again
  // Note: the stepper may have been stopped from an interrupt handler
  // (which is the case for tool detached), while normal execution was in
  // plan_buffer_line(). So, we check again, and remove the point we just
  // added, if necessary.
  const auto revertReason = stopReason();
  if (revertReason) {
    quickStop();
    logError
      << F("Unable to perform movement, ")
      << revertReason
      << endl;
    return -1;
  }

  // Success
  return 0;
}

void Stepper::outputStatus() {
  maxStepsComplete.outputStatus();
  maxInterruptsAllowed.outputStatus();
  maxCompletedWithoutTriggers.outputStatus();
  maxCompletedWithoutTriggersTics.outputStatus();
  maxStepRate.outputStatus();
  maxStepTiming.outputStatus();
}

void Stepper::periodicReport() {
  maxStepsComplete.reportIfChanged();
  maxInterruptsAllowed.reportIfChanged();
  maxCompletedWithoutTriggers.reportIfChanged();
  maxCompletedWithoutTriggersTics.reportIfChanged();
  maxStepRate.reportIfChanged();
  maxStepTiming.reportIfChanged();
}
