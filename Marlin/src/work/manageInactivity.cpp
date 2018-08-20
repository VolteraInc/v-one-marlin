#include "../../Marlin.h"
#include "../../planner.h"
#include "../api/api.h"
#include "../../macros.h"
#include "../../temperature_profile.h"
#include "../vone/VOne.h"

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
  static unsigned long nextCheckAt = 0;
  const auto now = millis();
  if (now < nextCheckAt) {
    return;
  }
  nextCheckAt = now + 1000;

  if((now - previous_millis_active_cmd) > stepper_inactive_time && stepper_inactive_time) {
    if(!blocks_queued()) {
      refresh_cmd_timeout(); // Reseting timeout stops us from constantly checking.
      log
        << F("The stepper has been inactive for more than ")
        << stepper_inactive_time
        << F("ms, deactivating motors")
        << endl;

      disable_x();
      disable_y();
      disable_z();
      disable_e();
      vone->toolBox.currentTool().resetPreparations();
    }
  }

  // If we've been waiting for ~1 hour. Kill the heater.
  if(now - previous_millis_serial_rx > heater_inactive_time && heater_inactive_time) {
    previous_millis_serial_rx = now; //Resetting timeout stops us from constantly checking
    log
      << F("No communication for more than ")
      << heater_inactive_time
      << F("ms, deactivating heater")
      << endl;
    profile_reset();
  }
}
