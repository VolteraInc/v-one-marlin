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
#include "speed_lookuptable.h"

#include "src/vone/VOne.h"
#include "src/vone/stepper/digipots.h"
#include "src/vone/endstops/EndstopMonitor.h"

//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced

//===========================================================================
//=============================private variables ============================
//===========================================================================

// Counter variables for the bresenham line tracer
static long counter_x, counter_y, counter_z, counter_e;

volatile static unsigned long step_events_completed; // The number of step events executed in the current block
static long acceleration_time, deceleration_time;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};

//===========================================================================
//=============================functions         ============================
//===========================================================================

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
asm volatile ( \
  "clr r26 \n\t" \
  "mul %A1, %B2 \n\t" \
  "movw %A0, r0 \n\t" \
  "mul %A1, %A2 \n\t" \
  "add %A0, r1 \n\t" \
  "adc %B0, r26 \n\t" \
  "lsr r0 \n\t" \
  "adc %A0, r26 \n\t" \
  "adc %B0, r26 \n\t" \
  "clr r1 \n\t" \
  : \
  "=&r" (intRes) \
  : \
  "d" (charIn1), \
  "d" (intIn2) \
  : \
  "r26" \
  )

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2) \
asm volatile ( \
  "clr r26 \n\t" \
  "mul %A1, %B2 \n\t" \
  "mov r27, r1 \n\t" \
  "mul %B1, %C2 \n\t" \
  "movw %A0, r0 \n\t" \
  "mul %C1, %C2 \n\t" \
  "add %B0, r0 \n\t" \
  "mul %C1, %B2 \n\t" \
  "add %A0, r0 \n\t" \
  "adc %B0, r1 \n\t" \
  "mul %A1, %C2 \n\t" \
  "add r27, r0 \n\t" \
  "adc %A0, r1 \n\t" \
  "adc %B0, r26 \n\t" \
  "mul %B1, %B2 \n\t" \
  "add r27, r0 \n\t" \
  "adc %A0, r1 \n\t" \
  "adc %B0, r26 \n\t" \
  "mul %C1, %A2 \n\t" \
  "add r27, r0 \n\t" \
  "adc %A0, r1 \n\t" \
  "adc %B0, r26 \n\t" \
  "mul %B1, %A2 \n\t" \
  "add r27, r1 \n\t" \
  "adc %A0, r26 \n\t" \
  "adc %B0, r26 \n\t" \
  "lsr r27 \n\t" \
  "adc %A0, r26 \n\t" \
  "adc %B0, r26 \n\t" \
  "clr r1 \n\t" \
  : \
  "=&r" (intRes) \
  : \
  "d" (longIn1), \
  "d" (longIn2) \
  : \
  "r26" , "r27" \
  )

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

  //         __________________________
  //        /|                        |\     _________________         ^
  //       / |                        | \   /|               |\        |
  //      /  |                        |  \ / |               | \       s
  //     /   |                        |   |  |               |  \      p
  //    /    |                        |   |  |               |   \     e
  //   +-----+------------------------+---+--+---------------+----+    e
  //   |               BLOCK 1            |      BLOCK 2          |    d
  //
  //                           time ----->
  //
  //  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
  //  first block->accelerate_until step_events_completed, then keeps going at constant speed until
  //  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
  //  The slope of acceleration is calculated with the leib ramp alghorithm.

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {

  if (step_rate > MAX_STEP_FREQUENCY) {
    step_rate = MAX_STEP_FREQUENCY;
  }

  if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2) & 0x3fff;
    step_loops = 4;
  } else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1) & 0x7fff;
    step_loops = 2;
  } else {
    step_loops = 1;
  }

  if (step_rate < (F_CPU/500000)) {
    step_rate = (F_CPU/500000);
  }
  step_rate -= (F_CPU/500000); // Correct for minimal speed

  unsigned short timer;
  if (step_rate >= (8*256)) { // higher step rate
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  } else { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }

  // 20kHz this should never happen
  if (timer < 100) {
    timer = 100;
    logWarning
      << F("Step rate ") << step_rate << F( " is too high ")
      << endl;
  }

  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;
}

// int idx = 0;
// unsigned int a[5] = {0};

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
void stepper_isr(EndstopMonitor& endstopMonitor) {
  static signed char xDir = 1;
  static signed char yDir = 1;
  static signed char zDir = 1;
  static signed char eDir = 1;

  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      current_block->busy = true;
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;

      // Set the direction
      const auto direction_bits = current_block->direction_bits;
      xDir = TEST(direction_bits, X_AXIS) ? -1 : 1;
      yDir = TEST(direction_bits, Y_AXIS) ? -1 : 1;
      zDir = TEST(direction_bits, Z_AXIS) ? -1 : 1;
      eDir = TEST(direction_bits, E_AXIS) ? -1 : 1;

      WRITE(X_DIR_PIN, xDir == -1 ? INVERT_X_DIR : !INVERT_X_DIR);
      WRITE(Y_DIR_PIN, yDir == -1 ? INVERT_Y_DIR : !INVERT_Y_DIR);
      WRITE(Z_DIR_PIN, zDir == -1 ? INVERT_Z_DIR : !INVERT_Z_DIR);
      WRITE(E_DIR_PIN, eDir == -1 ? INVERT_E_DIR : !INVERT_E_DIR);
    } else {
      OCR1A = 2000; // 1kHz.
    }
  }

  if (current_block != NULL) {

    // Detect end-stop hits
    // Note: only check in the direction(s) we are moving
    bool triggeredInX = false;
    bool triggeredInY = false;
    bool triggeredInZ = false;
    if (current_block->steps_x > 0) {
      endstopMonitor.onSteppingInX(xDir, count_position, triggeredInX);
    }
    if (current_block->steps_y > 0) {
      endstopMonitor.onSteppingInY(yDir, count_position, triggeredInY);
    }
    if (current_block->steps_z > 0) {
      endstopMonitor.onSteppingInZ(zDir, count_position, triggeredInZ);
    }

    if (triggeredInX || triggeredInY || triggeredInZ) {
      // Treat the block as complete
      // Note: Ensures timing is restored and current_block is removed
      step_events_completed = current_block->step_event_count;

      // Flush any remaining moves
      // Note: If this hit was expected then there should not be any more moves queued
      //       If it was not then any queued moves no longer make sense
      quickStop();
    } else {

      // Take multiple steps per interrupt (For high speed moves)
      for (int8_t i = 0; i < step_loops; i++) {
        counter_x += current_block->steps_x;
        if (counter_x > 0) {
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          counter_x -= current_block->step_event_count;
          count_position[X_AXIS] += xDir;
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        }

        counter_y += current_block->steps_y;
        if (counter_y > 0) {
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_y -= current_block->step_event_count;
          count_position[Y_AXIS] += yDir;
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        }

        counter_z += current_block->steps_z;
        if (counter_z > 0) {
          WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
          counter_z -= current_block->step_event_count;
          count_position[Z_AXIS] += zDir;
          WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
        }

        counter_e += current_block->steps_e;
        if (counter_e > 0) {
          WRITE(E_STEP_PIN, !INVERT_E_STEP_PIN);
          counter_e -= current_block->step_event_count;
          count_position[E_AXIS] += eDir;
          WRITE(E_STEP_PIN, INVERT_E_STEP_PIN);
        }

        step_events_completed += 1;
        if (step_events_completed >= current_block->step_event_count) {
          break;
        }
      }
    }

    // Calculate new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {

      MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      //acc_step_ = deceleration_time * current_block->acceleration_rate >> 24

      acc_step_rate += current_block->initial_rate;

      // upper limit
      if (acc_step_rate > current_block->nominal_rate) {
        acc_step_rate = current_block->nominal_rate;
      }

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;

    } else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {
      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
      // step_rate = deceleration_time * current_block->acceleration_rate >> 24
      if (step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      } else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if (step_rate < current_block->final_rate) {
        step_rate = current_block->final_rate;
      }

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;

    } else {
      OCR1A = OCR1A_nominal;
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }
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
    log << F("synchonized") << endl;
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
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (blocks_queued()) {
    plan_discard_current_block();
  }
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}
