/*
stepper.c - stepper motor driver: executes motion plans using stepper motors
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
and Philipp Tiefenbacher. */

#include "stepper.h"

#include "macros.h"
#include "Configuration.h"
#include "planner.h"

#include "src/vone/VOne.h"
#include "src/vone/stepper/digipots.h"
#include "src/vone/endstops/EndstopMonitor.h"
#include "src/vone/stepper/calculateStepTiming.h"

//===========================================================================
//=============================private variables ============================
//===========================================================================

static volatile bool s_stopRequested = false;

static long acceleration_time, deceleration_time;
static unsigned short acc_step_rate; // needed for deccelaration start point
static uint16_t OCR1A_nominal;
static uint8_t stepPerISR_nominal;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};

static signed char xDir = 1;
static signed char yDir = 1;
static signed char zDir = 1;
static signed char eDir = 1;

// Counter variables for the bresenham line tracer
static long counter_x, counter_y, counter_z, counter_e;

// The number of step events executed in the current block
static unsigned long step_events_completed = 0;

//===========================================================================
//=============================functions         ============================
//===========================================================================

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)
#define STEPPER_ISR_ENABLED()             TEST(TIMSK1, OCIE1A)

static FORCE_INLINE void s_handleNewBlock(
  const volatile block_t& block,
  uint16_t& timer,
  uint8_t& stepsPerISR
) {
  // Pre-compute values for nominal speed
  calculateStepTiming(block.nominal_rate, OCR1A_nominal, stepPerISR_nominal);

  // Reset acceleration variables
  deceleration_time = 0;
  acc_step_rate = block.initial_rate;
  calculateStepTiming(acc_step_rate, timer, stepsPerISR);
  vone->stepper.maxStepRate.update(acc_step_rate);
  acceleration_time = timer;

  // Reset step counters
  counter_x = -(block.step_event_count >> 1);
  counter_y = counter_x;
  counter_z = counter_x;
  counter_e = counter_x;
  step_events_completed = 0;

  // Set the direction
  const auto direction_bits = block.direction_bits;
  xDir = TEST(direction_bits, X_AXIS) ? -1 : 1;
  yDir = TEST(direction_bits, Y_AXIS) ? -1 : 1;
  zDir = TEST(direction_bits, Z_AXIS) ? -1 : 1;
  eDir = TEST(direction_bits, E_AXIS) ? -1 : 1;

  WRITE(X_DIR_PIN, xDir == -1 ? INVERT_X_DIR : !INVERT_X_DIR);
  WRITE(Y_DIR_PIN, yDir == -1 ? INVERT_Y_DIR : !INVERT_Y_DIR);
  WRITE(Z_DIR_PIN, zDir == -1 ? INVERT_Z_DIR : !INVERT_Z_DIR);
  WRITE(E_DIR_PIN, eDir == -1 ? INVERT_E_DIR : !INVERT_E_DIR);
}

static FORCE_INLINE void s_doStep(const volatile block_t& block) {
  counter_x += block.steps_x;
  if (counter_x > 0) {
    WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
    counter_x -= block.step_event_count;
    count_position[X_AXIS] += xDir;
    WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
  }

  counter_y += block.steps_y;
  if (counter_y > 0) {
    WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
    counter_y -= block.step_event_count;
    count_position[Y_AXIS] += yDir;
    WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
  }

  counter_z += block.steps_z;
  if (counter_z > 0) {
    WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
    counter_z -= block.step_event_count;
    count_position[Z_AXIS] += zDir;
    WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
  }

  counter_e += block.steps_e;
  if (counter_e > 0) {
    WRITE(E_STEP_PIN, !INVERT_E_STEP_PIN);
    counter_e -= block.step_event_count;
    count_position[E_AXIS] += eDir;
    WRITE(E_STEP_PIN, INVERT_E_STEP_PIN);
  }
}

static FORCE_INLINE bool s_checkEndstops(const volatile block_t& block) {
  bool triggeredInX = false;
  bool triggeredInY = false;
  bool triggeredInZ = false;
  auto& endstopMonitor = vone->stepper.endstopMonitor;
  if (block.steps_x > 0) {
    endstopMonitor.onSteppingInX(xDir, count_position, triggeredInX);
  }
  if (block.steps_y > 0) {
    endstopMonitor.onSteppingInY(yDir, count_position, triggeredInY);
  }
  if (block.steps_z > 0) {
    endstopMonitor.onSteppingInZ(zDir, count_position, triggeredInZ);
  }
  return triggeredInX || triggeredInY || triggeredInZ;
}

static FORCE_INLINE void s_calculateStepTiming(
  const volatile block_t& block,
  uint16_t& timer,
  uint8_t& stepsPerIsr
) {
  auto& stepper = vone->stepper;

  // --------------------------------------------
  // Calculate new timer value
  if (step_events_completed <= (unsigned long int)block.accelerate_until) {
    MultiU24X24toH16(acc_step_rate, acceleration_time, block.acceleration_rate);
    //acc_step_ = deceleration_time * block.acceleration_rate >> 24

    acc_step_rate += block.initial_rate;

    // upper limit
    if (acc_step_rate > block.nominal_rate) {
      acc_step_rate = block.nominal_rate;
    }

    // step_rate to timer interval
    stepper.maxStepRate.update(acc_step_rate);
    calculateStepTiming(acc_step_rate, timer, stepsPerIsr);
    acceleration_time += timer;
  } else if (step_events_completed > (unsigned long int)block.decelerate_after) {
    unsigned short step_rate;
    MultiU24X24toH16(step_rate, deceleration_time, block.acceleration_rate);
    // step_rate = deceleration_time * block.acceleration_rate >> 24
    if (step_rate > acc_step_rate) { // Check step_rate stays positive
      step_rate = block.final_rate;
    } else {
      step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
    }

    // lower limit
    if (step_rate < block.final_rate) {
      step_rate = block.final_rate;
    }

    // step_rate to timer interval
    stepper.maxStepRate.update(step_rate);
    calculateStepTiming(step_rate, timer, stepsPerIsr);
    deceleration_time += timer;
  } else {
    stepper.maxStepRate.update(block.nominal_rate);
    timer = OCR1A_nominal;
    stepsPerIsr = stepPerISR_nominal;
  }
}

ISR(TIMER1_COMPA_vect) {
  const auto isr_start = micros();
  auto& stepper = vone->stepper;
  static uint8_t step_loops = 1;
  static block_t *current_block = nullptr;

  // --------------------------------------------
  // Check for stop request
  if (s_stopRequested) {
    s_stopRequested = false;
    while (blocks_queued()) {
      plan_discard_current_block();
    }
    current_block = nullptr;
  }

  // --------------------------------------------
  // Move, if we already have a block
  if (current_block) {
    // Take multiple steps per interrupt (For high speed moves)
    for (int8_t i = 0; i < step_loops; i++) {
      s_doStep(*current_block);

      if (i == 0) {
        stepper.maxStepsComplete.update(micros() - isr_start);
      }

      step_events_completed += 1;
      if (step_events_completed >= current_block->step_event_count) {
        break;
      }
    }
  }

  // --------------------------------------------
  // Allow other interrupts, so we don't miss serial characters
  const bool temp_isr_was_enabled = TEMPERATURE_ISR_ENABLED();
  DISABLE_TEMPERATURE_INTERRUPT();
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  stepper.maxInterruptsAllowed.update(micros() - isr_start); // should be less than 40us, or we risk missing a character
  sei();

  // --------------------------------------------
  // Continue processing the current block
  auto triggered = false;
  if (current_block) {
    // Detect end-stop hits
    // Note: only check in the direction(s) we are moving
    triggered = s_checkEndstops(*current_block);
    if (triggered) {
      current_block = nullptr;

      // Flush any remaining moves
      // Note: If this hit was expected then there should not be any more moves queued
      //       If it was not then any queued moves no longer make sense
      quickStop();

    // If block finished normally
    } else if (step_events_completed >= current_block->step_event_count) {
      current_block = nullptr;
      plan_discard_current_block();
    }
  }

  // --------------------------------------------
  // Determine timing for next isr call
  uint16_t timer = 2000; // 1kHz
  if (current_block) {
    s_calculateStepTiming(*current_block, timer, step_loops);
  } else {
    // Get next block
    current_block = plan_get_current_block();
    if (current_block) {
      current_block->busy = true;
      s_handleNewBlock(*current_block, timer, step_loops);
    }
  }
  stepper.maxStepTiming.updateIfLower(timer);
  OCR1A = timer;

  // --------------------------------------------
  // Restore interrupt settings
  cli();
  ENABLE_STEPPER_DRIVER_INTERRUPT();
  if (temp_isr_was_enabled) ENABLE_TEMPERATURE_INTERRUPT();

  // --------------------------------------------
  // Reporting
  if (!triggered) {
    stepper.maxCompletedWithoutTriggers.update(micros() - isr_start);
    stepper.maxCompletedWithoutTriggersTics.update(TCNT1);
  }
}

void st_init() {
  // Initialize Digipot Motor Current
  digiPotInit();

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
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

// Block until all buffered steps are executed
void st_synchronize() {
  while (blocks_queued()) {
    periodic_work();
  }
  if (logging_enabled) {
    log << F("synchronized") << endl;
  }
}

void st_set_position(const long& x, const long& y, const long& z, const long& e) {
  ScopedInterruptDisable sid;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
}

void st_set_e_position(const long& e) {
  ScopedInterruptDisable sid;
  count_position[E_AXIS] = e;
}

long st_get_position(AxisEnum axis) {
  ScopedInterruptDisable sid;
  return count_position[axis];
}

float st_get_position_mm(AxisEnum axis) {
  return stepsToPositionInAxis(axis, count_position);
}

void quickStop() {
  ScopedInterruptDisable sid;
  s_stopRequested = true;
}
