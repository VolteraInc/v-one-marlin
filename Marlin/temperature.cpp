/*
  temperature.c - temperature control
  Part of Marlin

 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */

#include "Marlin.h"
#include "temperature.h"
#include "watchdog.h"
#include "macros.h"
#include "src/api/api.h" // Router::RampUpDuration

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature_bed = 0;
float current_temperature_bed = 0.0f;

float current_p_top = 0.0f;

unsigned char soft_pwm_bed;

//===========================================================================
//=============================private variables============================
//===========================================================================
static unsigned long COMMS_MODE_TIMEOUT = Router::RampUpDuration + 7500u; //ms
static volatile bool adc_samples_ready = false;
static int current_temperature_bed_raw = 0;
static int current_p_top_raw = 0;
static bool overlap_detected = false;
static bool p_top_in_comms_mode = false;;

// -----------------------------------------------
// adc management

void set_p_top_mode(enum PTopModes mode) {
  // Note: changing a form OUTPUT to INPUT generates noise for all analog reads
  // that's why we've encapsulate this code here
  bool comms_mode = false;
  switch(mode) {
    case P_TOP_COMMS_WRITE_MODE:
      SET_OUTPUT(ROUTER_COMMS_PIN);
      comms_mode = true;
      break;
    case P_TOP_COMMS_READ_MODE:
      SET_INPUT(ROUTER_COMMS_PIN);
      comms_mode = true;
      break;
    default:
      SET_INPUT(ROUTER_COMMS_PIN);
      comms_mode = false;
      break;
  }

  if (p_top_in_comms_mode != comms_mode) {
    static unsigned long start_time = 0;
    if (comms_mode) {
      start_time = millis();
      if (logging_enabled) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Entering p_top comms mode "); SERIAL_ECHOLN(start_time);
      }
    } else {
      if (logging_enabled) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Exiting p_top comms mode "); SERIAL_ECHOLN(millis());
      }
      const unsigned long duration = millis() - start_time;
      if (duration >= COMMS_MODE_TIMEOUT) {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("WARNING: Overlapping use of pin detected, pin was in use for ");
        SERIAL_ECHO(duration);
        SERIAL_ECHOLNPGM("ms");
      }
    }

    CRITICAL_SECTION_START;
    p_top_in_comms_mode = comms_mode;
    CRITICAL_SECTION_END;
  }
}

bool periodicAnalogReadsEnabled() {
  return !p_top_in_comms_mode;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
  float celsius = 0;
  byte i;

  #define PGM_RD_W(x)   (short)pgm_read_word(&x)

  for (i=1; i<BEDTEMPTABLE_LEN; i++)
  {
    if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw)
    {
      celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]) +
        (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) *
        (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /
        (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]));
      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == BEDTEMPTABLE_LEN) {
    celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]);
  }

  return celsius;
}

float rawToVoltage(int value) {
  return 5.0f * value / 1024.0f;
}

// Transfers the sampled, adc'd values from interrupt context into
// normal context
// Note: Comments in the original marlin code claim that this code
// is too slow to run in interrupts and will block the stepper routine
static void updateAdcValuesFromRaw() {
  if (adc_samples_ready) {
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
    current_p_top = rawToVoltage(current_p_top_raw / OVERSAMPLENR);

    CRITICAL_SECTION_START;
    adc_samples_ready = false;
    CRITICAL_SECTION_END;
  }
}

float get_p_top_voltage() {
  updateAdcValuesFromRaw();
  return current_p_top;
}

float next_p_top_voltage() {
  // If there are samples waiting, process them
  // Note: we don't know how stale they are, so we should not use them
  updateAdcValuesFromRaw();

  // Wait for a new set of samples
  while (!adc_samples_ready) {
    delay(10);
  }

  // return new voltage sample
  return get_p_top_voltage();
}

void manage_adc() {
  // Report overlapping use of p_top
  // Note: if this warning is seen then we need to allocate more time
  // or reduce then number of retries when using the pin for communication.
  if (overlap_detected) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("WARNING: Overlapping use of pin detected, any voltages reported during the overlap are invalid");

    // Reset so we can detect additional occurences
    CRITICAL_SECTION_START;
    overlap_detected = false;
    CRITICAL_SECTION_END;
  }

  // Update sampled values
  updateAdcValuesFromRaw();
}

// -----------------------------------------------
// Heater

int getSoftPwmBed() {
	return soft_pwm_bed;
}

void disable_heater() {
  setTargetBed(0);
  soft_pwm_bed = 0;
  WRITE(HEATER_BED_PIN, 0);
}

void manage_heater() {
  // Run periodically
  static unsigned long nextCheckAt = 0;
  const auto now = millis();
  if (now < nextCheckAt) {
    return;
  }
  nextCheckAt = now + BED_CHECK_INTERVAL;

  // Check if bed temperature exceed bounds
  if(degBed() <= BED_MINTEMP || degBed() >= BED_MAXTEMP) {
    disable_heater();
    return;
  }

  // Turn bed on/off as needed
  soft_pwm_bed = degBed() < degTargetBed() ? MAX_BED_POWER >> 1 : 0;
}

// -----------------------------------------------
// Module init and interrupt handler

void tp_init() {
  SET_OUTPUT(HEATER_BED_PIN);

  // Set analog inputs
  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
  DIDR0 = 0;
  DIDR2 = 0;
  DIDR0 |= 1<<TEMP_BED_PIN;

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millis interrupt
  OCR0B = 128;
  TIMSK0 |= (1<<OCIE0B);

  // Wait for temperature measurement to settle
  delay(250);
}

// Timer 0 is shared with millis
ISR(TIMER0_COMPB_vect) {

  //---------------------------------------------
  // soft PWM
  static unsigned char pwm_count = (1 << SOFT_PWM_SCALE);
  static unsigned char soft_pwm_b;

  if(pwm_count == 0){
    soft_pwm_b = soft_pwm_bed;
    if(soft_pwm_b > 0) {
      WRITE(HEATER_BED_PIN, 1);
    } else {
      WRITE(HEATER_BED_PIN, 0);
    }
  }
  if(soft_pwm_b < pwm_count) {
    WRITE(HEATER_BED_PIN, 0);
  }

  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;

  //---------------------------------------------
  // Temperature monitoring & other ADC reading
  // These state are used to manage use of the analog-digital converter (ADC).
  // The ADC must be setup to read a particular pin then we must wait before
  // retrieving the value. Waiting is implemented by exiting this ISR then
  // reading the ADC the next time it is called.
  enum AdcReadState {
    PrepareTemp_BED,
    MeasureTemp_BED,
    Prepare_P_TOP,
    Measure_P_TOP,
    StartupDelay, // Startup, delay initial reading a tiny bit so the hardware can settle
    TemporarilyDisabled
  };
  static enum AdcReadState adc_read_state = StartupDelay;
  static unsigned long no_adc_reads_until = 0;

  static unsigned char sample_count = 0;
  static unsigned long raw_bed_temp = 0;
  static unsigned long raw_p_top = 0;

  // Note: we use a timeout here as a safety mechanism.
  // i.e. a bug or unexpectedly long communication sequence
  // should not disable temperature management.
  // Also, we need a timeout after returning to read mode to allow
  // noise to settle out of the ADC (or all pins, not just P_TOP).
  if (p_top_in_comms_mode && adc_read_state != TemporarilyDisabled) {
    adc_read_state = TemporarilyDisabled;
    no_adc_reads_until = millis() + COMMS_MODE_TIMEOUT;

    // Reset samples
    sample_count = 0;
    raw_bed_temp = 0;
    raw_p_top = 0;
  }

  #define START_ADC(pin) ADCSRB = 0; ADMUX = _BV(REFS0) | (pin & 0x07); SBI(ADCSRA, ADSC)
  switch(adc_read_state) {

    case PrepareTemp_BED:
      START_ADC(TEMP_BED_PIN);
      adc_read_state = MeasureTemp_BED;
      break;
    case MeasureTemp_BED:
      raw_bed_temp += ADC;
      adc_read_state = Prepare_P_TOP;
      break;

    case Prepare_P_TOP:
      START_ADC(P_TOP_STATE_PIN); // same pin as P_TOP_PIN / ROUTER_COMMS_PIN
      adc_read_state = Measure_P_TOP;
      break;
    case Measure_P_TOP:
      raw_p_top += ADC;
      adc_read_state = PrepareTemp_BED;

      ++sample_count;
      break;

    case StartupDelay:
      adc_read_state = PrepareTemp_BED;
      break;

    case TemporarilyDisabled:
      // We can't wait forever, temperature readings are needed for safe operation
      if (millis() >= no_adc_reads_until || !p_top_in_comms_mode) {
        no_adc_reads_until = 0;
        adc_read_state = PrepareTemp_BED;

        // if still in comms mode report an error (if there isn't one already)
        if (p_top_in_comms_mode) {
          overlap_detected = true;
        }
      }
      break;
  }

  // One we collect enough samples, report values and reset
  // Note: Samples accumulate every 66ms (measured).
  // This matches the expected frequency: 4 states * 16 samples * 1/(16000000/64/256) ~= 66ms
  if (sample_count >= OVERSAMPLENR) {
    // Only update the shared values if they have been read
    // otherwise we could be updating them during reading.
    if (!adc_samples_ready) {
      current_temperature_bed_raw = raw_bed_temp;
      current_p_top_raw = raw_p_top;
      adc_samples_ready = true;
    }

    // Reset
    sample_count = 0;
    raw_bed_temp = 0;
    raw_p_top = 0;
  }
}
