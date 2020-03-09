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

static void s_autoShutOff_Motors(unsigned long now) {
  if (
    !vone->motors.anyEnabled() || // motors are already off
    stepper_inactive_time == 0 || // no timeout is configured
    now - previous_millis_active_cmd < stepper_inactive_time || // timeout has not been reached
    blocks_queued() // still work in queue
  ) {
    return;
  }

  // Reseting timeout
  // NOTE: stops us from constantly checking
//  refresh_cmd_timeout();

  log
    << F("The system has been inactive for more than ")
    << stepper_inactive_time
    << F("ms, deactivating motors")
    << endl;

  vone->toolBox.currentTool().resetPreparations();
  vone->motors.off();
}

static void s_autoShutOff_Heater(unsigned long now) {
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

void manage_inactivity() {
  // Run periodically
  static unsigned long nextCheckAt = 0;
  const auto now = millis();
  if (now < nextCheckAt) {
    return;
  }
  nextCheckAt = now + 1000;

  s_autoShutOff_Motors(now);
  s_autoShutOff_Heater(now);
}
