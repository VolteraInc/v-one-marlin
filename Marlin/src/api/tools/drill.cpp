#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"

static bool enabled = false; // Start with drill off.
static int feedrate = 0; // Start with no speed.
static unsigned long pulse_start = 0;
static bool pulse_high = false;
static float period_ms = 20;


void drill_enable(){
  SET_OUTPUT(DRILL_PIN);
  WRITE(DRILL_PIN,LOW);
  enabled = true;
}
void drill_disable() {
  SET_INPUT(DRILL_PIN); //Pin is used as an analog read.
  enabled = false;
  feedrate = 0;
}

void drill_set_speed(int new_feedrate){
  feedrate = new_feedrate;
}

float drill_set_frequency(float new_frequency) {
  period_ms = 1/new_frequency*1000;
  return period_ms;
}

void drill_monitor() {
  // Check if drill is ON - We implement a ~60Hz signal with PWM. Pulse width is between 1ms and 2ms
  if(enabled) {

      // Have we pulsed for long enough? If so, let's turn off the pulse.
      if (pulse_high && micros() - pulse_start > (unsigned long) 1000 * feedrate / 255 + 500) {
          pulse_high = false;
          WRITE(DRILL_PIN, false);
      }

      // If it's been off long enough, kick off another pulse.
      else if (micros() - pulse_start > (unsigned long) period_ms*1000){
        pulse_start = micros();
        pulse_high = true;
        WRITE(DRILL_PIN, true);
      }
  }

}
