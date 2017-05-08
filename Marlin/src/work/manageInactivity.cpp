#include "../../Configuration_adv.h"
#include "../../Marlin.h"
#include "../../temperature.h"
#include "../../planner.h"
#include "../api/api.h"
#include "../../macros.h"

unsigned long previous_millis_serial_rx = 0;
static unsigned long previous_millis_active_cmd = 0;

// After this long without serial traffic *and* no movement, everything shuts down
static unsigned long heater_inactive_time = DEFAULT_HEATER_DEACTIVE_TIME * 1000l;
unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME * 1000l;

void refresh_cmd_timeout() {
  previous_millis_active_cmd = millis();
}

void refresh_serial_rx_timeout() {
  previous_millis_serial_rx = millis();
}

void setStepperInactiveDuration(unsigned long duration) {
  stepper_inactive_time = duration;
}

void manage_inactivity() {
  // Run periodically
  static auto nextCheckAt = 0u;
  const auto now = millis();
  if (now < nextCheckAt) {
    return;
  }
  nextCheckAt = now + 1000;

  if((now - previous_millis_active_cmd) > stepper_inactive_time && stepper_inactive_time) {
    if(!blocks_queued()) {
      refresh_cmd_timeout(); // Reseting timeout stops us from constantly checking.
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("The stepper has been inactive for more than "); SERIAL_ECHO(stepper_inactive_time);
      SERIAL_ECHOPGM("ms, deactivating motors\n");
      disable_x();
      disable_y();
      disable_z();
      disable_e0();
      resetToolPreparations();
    }
  }

  // If we've been waiting for ~1 hour. Kill the heater.
  if(now - previous_millis_serial_rx > heater_inactive_time && heater_inactive_time) {
    previous_millis_serial_rx = now; //Resetting timeout stops us from constantly checking
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("No communication for more than "); SERIAL_ECHO(heater_inactive_time);
    SERIAL_ECHOPGM("ms, deactivating heater\n");
    disable_heater();
  }
}
