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

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature_bed = 0;

int current_temperature_bed_raw = 0;
float current_temperature_bed = 0.0;

#ifdef PIDTEMPBED
  float bedKp=DEFAULT_bedKp;
  float bedKi=(DEFAULT_bedKi*PID_dT);
  float bedKd=(DEFAULT_bedKd/PID_dT);
#endif //PIDTEMPBED

unsigned char soft_pwm_bed;

//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

#ifdef PIDTEMPBED
  //static cannot be external:
  static float temp_iState_bed = { 0 };
  static float temp_dState_bed = { 0 };
  static float pTerm_bed;
  static float iTerm_bed;
  static float dTerm_bed;
  //int output;
  static float pid_error_bed;
  static float temp_iState_min_bed;
  static float temp_iState_max_bed;
#else //PIDTEMPBED
	static unsigned long  previous_millis_bed_heater;
#endif //PIDTEMPBED
  static unsigned char soft_pwm[EXTRUDERS];

// Init min and max temp with extreme values to prevent false errors during startup
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;

static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

//===========================================================================
//=============================   functions      ============================
//===========================================================================

void PID_autotune(float temp, int extruder, int ncycles) {
  float input = 0.0;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

  if (extruder != -1){
    SERIAL_ECHOLNPGM("PID Autotune failed. Bad extruder number.");
    return;
  }

  SERIAL_ECHOLNPGM("PID Autotune start");

  disable_heater(); // switch off all heaters.

  if (extruder<0)
  {
     soft_pwm_bed = (MAX_BED_POWER)/2;
     bias = d = (MAX_BED_POWER)/2;
   }
   else
   {
     soft_pwm[extruder] = (PID_MAX)/2;
     bias = d = (PID_MAX)/2;
  }

 for(;;) {
    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = current_temperature_bed;

      max=max(max,input);
      min=min(min,input);
      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) {
          heating=false;
          if (extruder<0)
            soft_pwm_bed = (bias - d) >> 1;
          else
            soft_pwm[extruder] = (bias - d) >> 1;
          t1=millis();
          t_high=t1 - t2;
          max=temp;
        }
      }
      if(heating == false && input < temp) {
        if(millis() - t1 > 5000) {
          heating=true;
          t2=millis();
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,(extruder<0?(MAX_BED_POWER):(PID_MAX))-20);
            if(bias > (extruder<0?(MAX_BED_POWER):(PID_MAX))/2) d = (extruder<0?(MAX_BED_POWER):(PID_MAX)) - 1 - bias;
            else d = bias;

            SERIAL_PROTOCOLPGM(" bias: "); SERIAL_PROTOCOL(bias);
            SERIAL_PROTOCOLPGM(" d: "); SERIAL_PROTOCOL(d);
            SERIAL_PROTOCOLPGM(" min: "); SERIAL_PROTOCOL(min);
            SERIAL_PROTOCOLPGM(" max: "); SERIAL_PROTOCOLLN(max);
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              SERIAL_PROTOCOLPGM(" Ku: "); SERIAL_PROTOCOL(Ku);
              SERIAL_PROTOCOLPGM(" Tu: "); SERIAL_PROTOCOLLN(Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              SERIAL_PROTOCOLLNPGM(" Classic PID ");
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              /*
              Kp = 0.33*Ku;
              Ki = Kp/Tu;
              Kd = Kp*Tu/3;
              SERIAL_PROTOCOLLNPGM(" Some overshoot ")
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              Kp = 0.2*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/3;
              SERIAL_PROTOCOLLNPGM(" No overshoot ")
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              */
            }
          }
          if (extruder<0)
            soft_pwm_bed = (bias + d) >> 1;
          else
            soft_pwm[extruder] = (bias + d) >> 1;
          cycles++;
          min=temp;
        }
      }
    }
    if(input > (temp + 20)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! Temperature too high");
      return;
    }
    if(millis() - temp_millis > 2000) {
      int p;
      if (extruder<0){
        p=soft_pwm_bed;
        SERIAL_PROTOCOLPGM(" B:");
      }else{
        p=soft_pwm[extruder];
        SERIAL_PROTOCOLPGM(" T:");
      }

      SERIAL_PROTOCOL(input);
      SERIAL_PROTOCOLPGM(" @:");
      SERIAL_PROTOCOLLN(p);

      temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! timeout");
      return;
    }
    if(cycles > ncycles) {
      SERIAL_PROTOCOLLNPGM("PID Autotune finished! Put the last Kp, Ki and Kd constants from above into Configuration.h");
      return;
    }
  }
}

void updatePID() {
#ifdef PIDTEMPBED
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
#endif
}

int getHeaterPower(int heater) {
	if (heater<0)
		return soft_pwm_bed;
  return soft_pwm[heater];
}

void manage_heater() {
  if(temp_meas_ready != true)   //better readability
    return;

  updateTemperaturesFromRawValues();

  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();

  // Check if bed temperature is within the correct range
  if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP)) {
    if(current_temperature_bed >= target_temperature_bed) {
      soft_pwm_bed = 0;
    } else {
      soft_pwm_bed = MAX_BED_POWER>>1;
    }
  } else {
    soft_pwm_bed = 0;
    WRITE(HEATER_BED_PIN, 0);
  }
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
  float celsius = 0;
  byte i;

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

// Called to get the raw values into the actual temperatures.
// The raw values are created in interrupt context, and this
// function is called from normal context as it is too slow to
// run in interrupts and will block the stepper routine
static void updateTemperaturesFromRawValues() {
  current_temperature_bed = analog2tempBed(current_temperature_bed_raw);

  CRITICAL_SECTION_START;
  temp_meas_ready = false;
  CRITICAL_SECTION_END;
}

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

  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
    #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
      bed_maxttemp_raw -= OVERSAMPLENR;
    #else
      bed_maxttemp_raw += OVERSAMPLENR;
    #endif
  }
}

void disable_heater() {
  setTargetBed(0);
  soft_pwm_bed = 0;
  WRITE(HEATER_BED_PIN, 0);
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
    StartupDelay // Startup, delay initial reading a tiny bit so the hardware can settle
  };
  static unsigned char adc_read_state = StartupDelay;

  static unsigned char sample_count = 0;
  static unsigned long raw_bed_temp = 0;
  static unsigned long raw_p_top = 0;

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
      START_ADC(P_TOP_STATE_PIN);
      adc_read_state = Measure_P_TOP;
      break;
    case Measure_P_TOP:
      raw_p_top += ADC;
      adc_read_state = PrepareTemp_BED;
      ++sample_count;
      break;

    case StartupDelay:
      adc_read_state = 0;
      break;
  }

  if(sample_count >= OVERSAMPLENR) // 8 * 16 * 1/(16000000/64/256)  = 131ms.
  {
    // Only update the raw values if they have been read. Else we could be updating them during reading.
    if (!temp_meas_ready) {
      current_temperature_raw[0] = 0;
      current_temperature_bed_raw = raw_bed_temp;
    }

    temp_meas_ready = true;
    sample_count = 0;
    raw_bed_temp = 0;
    raw_p_top = 0;
  }
}

#ifdef PIDTEMPBED
// Apply the scale factors to the PID values

float scalePID_i(float i) {
	return i * PID_dT;
}

float unscalePID_i(float i) {
	return i / PID_dT;
}

float scalePID_d(float d) {
    return d / PID_dT;
}

float unscalePID_d(float d) {
	return d * PID_dT;
}

#endif //PIDTEMPBED
