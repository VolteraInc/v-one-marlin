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
int target_temperature[EXTRUDERS] = { 0 };
int target_temperature_bed = 0;
int current_temperature_raw[EXTRUDERS] = { 0 };
float current_temperature[EXTRUDERS] = { 0.0 };
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

# define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }

// Init min and max temp with extreme values to prevent false errors during startup
// static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP );
// static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP );
static int minttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );
//static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP; /* No bed mintemp error implemented?!? */
#ifdef BED_MAXTEMP
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#ifdef WATCH_TEMP_PERIOD
int watch_start_temp[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0);
unsigned long watchmillis[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0);
#endif //WATCH_TEMP_PERIOD

#ifndef SOFT_PWM_SCALE
#define SOFT_PWM_SCALE 0
#endif

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

  if ((extruder > EXTRUDERS)
  #if (TEMP_BED_PIN <= -1)
       ||(extruder < 0)
  #endif
       ){
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

      input = (extruder<0)?current_temperature_bed:current_temperature[extruder];

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
  float pid_input;
  float pid_output;

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

  //Reset the watchdog after we know we have a temperature measurement.
  watchdog_reset();

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

  #ifdef BED_MAXTEMP
    while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_maxttemp_raw -= OVERSAMPLENR;
      #else
        bed_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
  #endif //BED_MAXTEMP
}

void setWatch() {
#ifdef WATCH_TEMP_PERIOD
  for (int e = 0; e < EXTRUDERS; e++)
  {
    if(degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp[e] = degHotend(e);
      watchmillis[e] = millis();
    }
  }
#endif
}

void disable_heater() {
  for(int i=0;i<EXTRUDERS;i++)
    setTargetHotend(0,i);
  setTargetBed(0);

  soft_pwm_bed = 0;
  WRITE(HEATER_BED_PIN, 0);
}

void bed_max_temp_error(void) {
  if(!IsStopped()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Temperature heated bed switched off. MAXTEMP triggered !!");
  }
  Stop();
}


// Timer 0 is shared with millis
ISR(TIMER0_COMPB_vect) {
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned long raw_temp_bed_value = 0;
  static unsigned char temp_state = 8;

  //---------------------------------------------
  // soft PWM
  static unsigned char pwm_count = (1 << SOFT_PWM_SCALE);
  static unsigned char soft_pwm_0;
  static unsigned char soft_pwm_b;

  if(pwm_count == 0){
    soft_pwm_0 = soft_pwm[0];
    if(soft_pwm_0 > 0) {
      WRITE(HEATER_0_PIN,1);
    } else WRITE(HEATER_0_PIN,0);

    #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
    soft_pwm_b = soft_pwm_bed;
    if(soft_pwm_b > 0) WRITE(HEATER_BED_PIN,1); else WRITE(HEATER_BED_PIN,0);
    #endif
  }
  if(soft_pwm_0 < pwm_count) {
    WRITE(HEATER_0_PIN,0);
  }

  #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
  if(soft_pwm_b < pwm_count) WRITE(HEATER_BED_PIN,0);
  #endif

  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;

  //---------------------------------------------
  // Temperature monitoring & other ADC reading
  switch(temp_state) {
    case 0: // Prepare TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 1;
      break;
    case 1: // Measure TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        raw_temp_0_value += ADC;
      #endif
      temp_state = 2;
      break;
    case 2: // Prepare TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        #if TEMP_BED_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 3;
      break;
    case 3: // Measure TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        raw_temp_bed_value += ADC;
      #endif
      temp_state = 4;
      break;
    case 4: // Prepare TEMP_1
      #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
        #if TEMP_1_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_1_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 5;
      break;
    case 5: // Measure TEMP_1
      #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
        raw_temp_1_value += ADC;
      #endif
      temp_state = 6;
      break;
    case 6: // Prepare TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
        #if TEMP_2_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_2_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 7;
      break;
    case 7: // Measure TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
        raw_temp_2_value += ADC;
      #endif
      temp_state = 0;
      temp_count++;
      break;
    case 8: //Startup, delay initial temp reading a tiny bit so the hardware can settle.
      temp_state = 0;
      break;
  }

  if(temp_count >= OVERSAMPLENR) // 8 * 16 * 1/(16000000/64/256)  = 131ms.
  {
    // Only update the raw values if they have been read. Else we could be updating them during reading.
    if (!temp_meas_ready) {
      current_temperature_raw[0] = 0;
      current_temperature_bed_raw = raw_temp_bed_value;
    }

    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
    raw_temp_bed_value = 0;


  /* No bed MINTEMP error? */
# if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
    if(current_temperature_bed_raw <= bed_maxttemp_raw) {
#else
    if(current_temperature_bed_raw >= bed_maxttemp_raw) {
#endif
       bed_max_temp_error();
    }
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
