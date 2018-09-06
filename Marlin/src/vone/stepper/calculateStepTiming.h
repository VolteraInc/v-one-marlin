#pragma once

#include "../../../Configuration.h"
#include "../../../macros.h"
#include "../../../serial.h"
#include "speed_lookuptable.h"

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

FORCE_INLINE void calculateStepTiming(
  uint16_t step_rate,
  uint16_t& o_timer,
  uint8_t& o_stepsPerISR
) {

  if (step_rate > MAX_STEP_FREQUENCY) {
    step_rate = MAX_STEP_FREQUENCY;
  }

  if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2) & 0x3fff;
    o_stepsPerISR = 4;
  } else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1) & 0x7fff;
    o_stepsPerISR = 2;
  } else {
    o_stepsPerISR = 1;
  }

  if (step_rate < (F_CPU/500000)) {
    step_rate = (F_CPU/500000);
  }
  step_rate -= (F_CPU/500000); // Correct for minimal speed

  if (step_rate >= (8*256)) { // higher step rate
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(o_timer, tmp_step_rate, gain);
    o_timer = (unsigned short)pgm_read_word_near(table_address) - o_timer;
  } else { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    o_timer = (unsigned short)pgm_read_word_near(table_address);
    o_timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }

  // 20kHz this should never happen
  if (o_timer < 100) {
    o_timer = 100;
    logWarning
      << F("Step rate ") << step_rate << F( " is too high ")
      << endl;
  }
}