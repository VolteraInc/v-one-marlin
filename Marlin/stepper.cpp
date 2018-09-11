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

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#ifdef TRINAMIC_DRIVERS
#include <TMC2130Stepper.h>
#endif

#include "src/vone/VOne.h"

//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced

//===========================================================================
//=============================private variables ============================
//===========================================================================

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output

// Counter variables for the bresenham line tracer
static long counter_x, counter_y, counter_z, counter_e;

volatile static unsigned long step_events_completed; // The number of step events executed in the current block
static long acceleration_time, deceleration_time;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;

volatile long endstops_trigsteps[4]={0,0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;
static volatile bool endstop_e_hit=false;

//static bool old_x_min_endstop=false;
//static bool old_x_max_endstop=false;
//static bool old_y_min_endstop=false;
//static bool old_y_max_endstop=false;
static bool old_z_min_endstop=false;
static bool old_z_max_endstop=false;

static volatile bool p_top_enabled = false;
static volatile bool calibration_plate_enabled = false;
static volatile bool max_e_endstops_enabled = false;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

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

bool endstop_triggered(int axis) {
  bool triggered = false;
  CRITICAL_SECTION_START;
    switch (axis) {
      case X_AXIS: triggered = endstop_x_hit; break;
      case Y_AXIS: triggered = endstop_y_hit; break;
      case Z_AXIS: triggered = endstop_z_hit; break;
      case E_AXIS: triggered = endstop_e_hit; break;
    }
  CRITICAL_SECTION_END;
  return triggered;
}

/// Returns true if an endstop was hit
bool readAndResetEndstops(bool triggered[4], long stepsWhenTriggered[4]) {
  CRITICAL_SECTION_START;
    // Read and reset hit flags
    triggered[ X_AXIS ] = endstop_x_hit;
    triggered[ Y_AXIS ] = endstop_y_hit;
    triggered[ Z_AXIS ] = endstop_z_hit;
    triggered[ E_AXIS ] = endstop_e_hit;

    endstop_x_hit = false;
    endstop_y_hit = false;
    endstop_z_hit = false;
    endstop_e_hit = false;

    // Read and reset stepper location when triggered
    stepsWhenTriggered[ X_AXIS ] = endstops_trigsteps[ X_AXIS ];
    stepsWhenTriggered[ Y_AXIS ] = endstops_trigsteps[ Y_AXIS ];
    stepsWhenTriggered[ Z_AXIS ] = endstops_trigsteps[ Z_AXIS ];
    stepsWhenTriggered[ E_AXIS ] = endstops_trigsteps[ E_AXIS ];
    endstops_trigsteps[ X_AXIS ] = 0;
    endstops_trigsteps[ Y_AXIS ] = 0;
    endstops_trigsteps[ Z_AXIS ] = 0;
    endstops_trigsteps[ E_AXIS ] = 0;
  CRITICAL_SECTION_END;

  return (
    triggered[ X_AXIS ] ||
    triggered[ Y_AXIS ] ||
    triggered[ Z_AXIS ] ||
    triggered[ E_AXIS ]
  );
}

void clear_endstop(int axis) {
  CRITICAL_SECTION_START;
    switch (axis) {
      case X_AXIS: endstop_x_hit = false; break;
      case Y_AXIS: endstop_y_hit = false; break;
      case Z_AXIS: endstop_z_hit = false; break;
      case E_AXIS: endstop_e_hit = false; break;
    }
  CRITICAL_SECTION_END;
}

void enable_p_top(bool enable) {
  CRITICAL_SECTION_START;
    p_top_enabled = enable;
  CRITICAL_SECTION_END;
}

void enable_calibration_plate(bool enable) {
  CRITICAL_SECTION_START;
    calibration_plate_enabled = enable;
  CRITICAL_SECTION_END;
}

void enable_e_max_endstops(bool enable) {
  CRITICAL_SECTION_START;
    max_e_endstops_enabled = enable;
  CRITICAL_SECTION_END;
}

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

void st_wake_up() {
  // TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

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


FORCE_INLINE bool readNegX() {
  #ifdef TRINAMIC_SENSORLESS
    return READ_PIN(XY_MIN_X) || READ_PIN(X_LIM); // X- direction, also monitors XY_min_X
  #else
    return READ_PIN(XY_MIN_X) || READ_PIN(X_MIN); // X- direction, also monitors XY_min_X
  #endif
}

FORCE_INLINE bool readPosX() {
    return READ_PIN(XY_MAX_X);
}

FORCE_INLINE bool readNegY() {
  #ifdef TRINAMIC_SENSORLESS
  return READ_PIN(XY_MIN_Y) || READ_PIN(Y_LIM); // X- direction, also monitors XY_min_X
  #else
  return READ_PIN(XY_MIN_Y) || READ_PIN(Y_MIN); // X- direction, also monitors XY_min_X
  #endif
}

FORCE_INLINE bool readPosY() {
    return READ_PIN(XY_MAX_Y);
}

FORCE_INLINE bool readE() {
  #ifdef TRINAMIC_SENSORLESS
    return max_e_endstops_enabled ? READ_PIN(E_LIM) : false;
  #else
    return false;
  #endif
}



// int idx = 0;
// unsigned int a[5] = {0};

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
ISR(TIMER1_COMPA_vect) {

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
    } else {
      OCR1A = 2000; // 1kHz.
    }
  }

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;

    // Set direction to check limit switches / endstops.
    // Note: adjustments made here to monitor the correct XY-positioner limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
      WRITE(X_DIR_PIN, INVERT_X_DIR);
      count_direction[X_AXIS] = -1;

      bool x_min_endstop = readNegX();
      if (x_min_endstop && (current_block->steps_x > 0)) {
        endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
        endstop_x_hit = true;
        step_events_completed = current_block->step_event_count;
      }
      //old_x_min_endstop = x_min_endstop;
    } else { // +direction
      WRITE(X_DIR_PIN, !INVERT_X_DIR);
      count_direction[X_AXIS] = 1;

      bool x_max_endstop = readPosX();
      if (x_max_endstop && (current_block->steps_x > 0)){
        endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
        endstop_x_hit = true;
        step_events_completed = current_block->step_event_count;
      }
      //old_x_max_endstop = x_max_endstop;
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
      WRITE(Y_DIR_PIN, INVERT_Y_DIR);
      count_direction[Y_AXIS] = -1;

      bool y_min_endstop = readNegY();
      if (y_min_endstop && (current_block->steps_y > 0)) {
        endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
        endstop_y_hit = true;
        step_events_completed = current_block->step_event_count;
      }
      //old_y_min_endstop = y_min_endstop;
    } else { // +direction
      WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
      count_direction[Y_AXIS] = 1;

      bool y_max_endstop = readPosY();
      if (y_max_endstop && (current_block->steps_y > 0)){
        endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
        endstop_y_hit = true;
        step_events_completed = current_block->step_event_count;
      }
      //old_y_max_endstop = y_max_endstop;
    }

    if ((out_bits & (1<<Z_AXIS)) != 0) { // -direction
      WRITE(Z_DIR_PIN, INVERT_Z_DIR);
      count_direction[Z_AXIS] = -1;

      bool p_top = false;
      if (p_top_enabled) {
        if (vone->pins.ptop.readDigitalValue(p_top)) {
          // Treat read failures as TRIGGERED, so that motion stops
          // Note: not sure if this is the right thing to do, but it's
          // safer than allowing motion to continue. Also, if reads are
          // failing there is a bug
          p_top = true;
        }
      }

      bool z_min = READ_PIN(Z_MIN);
      bool p_bot = calibration_plate_enabled ? READ_PIN(P_BOT) : false;

      bool z_min_endstop = z_min || p_top || p_bot;
      if (z_min_endstop) {
        // a[idx++] = count_position[Z_AXIS];
        if (z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
          endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
          endstop_z_hit = true;
          step_events_completed = current_block->step_event_count;

          // log
          //   << "z_min" << z_min
          //   << " calibration_plate_enabled" << calibration_plate_enabled
          //   << " p_bot" << p_bot
          //   << " p_top_enabled" << p_top_enabled
          //   << " p_top" << p_top
          //   << endl;

          // for (unsigned int i = 0; i < 5; ++i) {
          //   log << "a[" << i << "] = " << a[i] << ", " << endl;
          //   a[i] = 0;
          // }
          // idx = 0;
        }
      }
      old_z_min_endstop = z_min_endstop;

    } else { // +direction
      WRITE(Z_DIR_PIN, !INVERT_Z_DIR);
      count_direction[Z_AXIS] = 1;

      bool z_max_endstop = READ_PIN(Z_MAX);
      if (z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
        endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
        endstop_z_hit = true;
        step_events_completed = current_block->step_event_count;
      }
      old_z_max_endstop = z_max_endstop;
    }

    if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
      WRITE(E_DIR_PIN, INVERT_E_DIR);
      count_direction[E_AXIS] = -1;

      bool e_min_endstop = readE();
      if(e_min_endstop && (current_block->steps_e > 0)) {
        endstops_trigsteps[E_AXIS] = count_position[E_AXIS];
        endstop_e_hit = true;
        step_events_completed = current_block->step_event_count;
      }
    } else { // +direction
      WRITE(E_DIR_PIN, !INVERT_E_DIR);
      count_direction[E_AXIS] = 1;

      bool e_min_endstop = readE();
      if(e_min_endstop && (current_block->steps_e  > 0)) {
        endstops_trigsteps[E_AXIS] = count_position[E_AXIS];
        endstop_e_hit = true;
        step_events_completed = current_block->step_event_count;
      }
    }

    for (int8_t i = 0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves)
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
        counter_x -= current_block->step_event_count;
        count_position[X_AXIS]+=count_direction[X_AXIS];
        WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
        counter_y -= current_block->step_event_count;
        count_position[Y_AXIS]+=count_direction[Y_AXIS];
        WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
      }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        WRITE_Z_STEP(!INVERT_Z_STEP_PIN);
        counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
        WRITE_Z_STEP(INVERT_Z_STEP_PIN);
      }

      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        WRITE_E_STEP(!INVERT_E_STEP_PIN);
        counter_e -= current_block->step_event_count;
        count_position[E_AXIS]+=count_direction[E_AXIS];
        WRITE_E_STEP(INVERT_E_STEP_PIN);
      }

      step_events_completed += 1;
      if (step_events_completed >= current_block->step_event_count) {
        break;
      }
    }

    // Calculare new timer value
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

  // Initialize Trinamic Driver.
  #ifdef TRINAMIC_DRIVERS
  trinamicInit(); // trinamicInit initalizes all the pins for us.
  #else
  // Initialize Digipot Motor Current
  digiPotInit();

  //Initialize Dir Pins
  SET_OUTPUT(X_DIR_PIN);
  SET_OUTPUT(Y_DIR_PIN);
  SET_OUTPUT(Z_DIR_PIN);
  SET_OUTPUT(E_DIR_PIN);

  // Initialize Enable Pins - steppers default to disabled.
  SET_OUTPUT(X_ENABLE_PIN);  if (!X_ENABLE_ON) { WRITE(X_ENABLE_PIN,  HIGH); }
  SET_OUTPUT(Y_ENABLE_PIN);  if (!Y_ENABLE_ON) { WRITE(Y_ENABLE_PIN,  HIGH); }
  SET_OUTPUT(Z_ENABLE_PIN);  if (!Z_ENABLE_ON) { WRITE(Z_ENABLE_PIN,  HIGH); }
  SET_OUTPUT(E_ENABLE_PIN); if (!E_ENABLE_ON) { WRITE(E_ENABLE_PIN, HIGH); }
  #endif
  // Endstops and pullups
  SET_INPUT(X_MIN_PIN);
  SET_INPUT(Y_MIN_PIN);
  SET_INPUT(Z_MIN_PIN);
  SET_INPUT(Z_MAX_PIN);

  // Endstops and pullups
  #ifdef TRINAMIC_SENSORLESS
  SET_INPUT(X_LIM_PIN);
  SET_INPUT(Y_LIM_PIN);
  SET_INPUT(E_LIM_PIN);
  #endif
  SET_INPUT(P_BOT_PIN);


  SET_INPUT(XY_MIN_X_PIN);
  SET_INPUT(XY_MAX_X_PIN);
  SET_INPUT(XY_MIN_Y_PIN);
  SET_INPUT(XY_MAX_Y_PIN);

  // LED Pins
  SET_OUTPUT(LED_RED_PIN);
  WRITE(LED_RED_PIN, LOW);

  SET_OUTPUT(LED_GREEN_PIN);
  WRITE(LED_GREEN_PIN, LOW);

  SET_OUTPUT(LED_BLUE_PIN);
  WRITE(LED_BLUE_PIN, LOW);

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
  CRITICAL_SECTION_START;
    count_position[X_AXIS] = x;
    count_position[Y_AXIS] = y;
    count_position[Z_AXIS] = z;
    count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const long& e) {
  CRITICAL_SECTION_START;
    count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis) {
  long count_pos;
  CRITICAL_SECTION_START;
    count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

float st_get_position_mm(uint8_t axis) {
  // To get our actual global position in mm based on our number of steps
  // we need to consider X and Y axis and do reverse transformations
  float position_mm = 0.0;
  if (axis == X_AXIS) {
    float steps_x = st_get_position(X_AXIS);
    position_mm = steps_x * calib_x_scale * calib_cos_theta / axis_steps_per_unit[X_AXIS];

  } else if (axis == Y_AXIS) {
    float steps_x = st_get_position(X_AXIS);
    float steps_y = st_get_position(Y_AXIS);
    position_mm = calib_x_scale * steps_x * sin(atan(calib_tan_theta)) / axis_steps_per_unit[X_AXIS] + calib_y_scale * steps_y / axis_steps_per_unit[Y_AXIS];

  } else { // Same case for both Z and E
    position_mm = st_get_position(axis) / axis_steps_per_unit[axis];
  }

  return position_mm;
}

void quickStop() {
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (blocks_queued()) {
    plan_discard_current_block();
  }
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

#ifdef TRINAMIC_DRIVERS

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

const TMC2130Stepper TMC2130[4] = { TMC2130_x, TMC2130_y, TMC2130_z, TMC2130_e };

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
  TMC2130[X_AXIS].stealth_max_speed(0); // TPWMTHRS - Upper velocity threshold for stealChop //250

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

#else

void digiPotInit() { //Initialize Digipot Motor Current
  const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;
  SPI.begin();
  pinMode(DIGIPOTSS_PIN, OUTPUT);

    // Cycle the SS pin - make sure digiPot initializes correctly.
  digitalWrite(DIGIPOTSS_PIN,LOW);
  digitalWrite(DIGIPOTSS_PIN,HIGH);

  for (int i = 0; i < 4; ++i) {
    digiPotSetCurrent(i, digipot_motor_current[i]);
  }
}

void digiPotSetCurrent(uint8_t axis, uint8_t current) {
  const uint8_t digipot_addrs[] = DIGIPOT_ADDRESS;
  digiPotWrite(digipot_addrs[axis], current);
}

uint8_t digiPotGetCurrent(uint8_t axis) {
  const uint8_t digipot_addrs[] = DIGIPOT_ADDRESS;
  return digiPotRead(digipot_addrs[axis]);
}

// Refer to http://ww1.microchip.com/downloads/en/DeviceDoc/22242A.pdf
// To get understanding of write/read codes and addresses.

void digiPotWrite(uint8_t address, uint8_t value) {
  address = address + 0x00; // 0x00 adds 'write' code. (no action here...)
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
  SPI.transfer(address); //  send in the address and value via SPI:
  SPI.transfer(value);
  digitalWrite(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
  SPI.endTransaction();
}

uint8_t digiPotRead(uint8_t address) {
  address =  address + 0x0C; // 0x0C adds 'read' code.
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
  SPI.transfer(address);
  uint8_t val = SPI.transfer(0x00); // Send dummy to clock in value.:
  digitalWrite(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
  SPI.endTransaction();
  return val;
}
#endif // TRINAMIC DRIVERS.
