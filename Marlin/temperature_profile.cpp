#include "Configuration.h"
#include "serial.h"
#include "src/vone/VOne.h"
#include "src/vone/bed/heater/Heater.h"
#include "src/utils/time.h"

#define PROFILE_SIZE (10)
static struct {
  unsigned long holdUntil = 0;
  unsigned long startTime = 0;
  float changeTemperature = 0;
  bool ramping = false;
  bool holding = false;
  int head = 0; // Indicates start of queue.
  int tail = 0; // Indicates end of queue
  int temperature[PROFILE_SIZE]; // Temperature in degrees C
  unsigned long duration[PROFILE_SIZE]; // Duration in seconds
} profile;


static int s_validate_input(const int temperature, const int duration){

  // Ensure both parameters were received.
  if (temperature == 0 || duration == 0){
    logError
      << F("Cannot interpret heating profile. Temperature: ") << temperature
      << F(" Duration: ") << duration
      << endl;
    return -1;
  }

  if (temperature < 0 || temperature > 240){
    logError
      << F("Invalid temperature target received: ") << temperature
      << endl;
    return -1;
  }

  if (duration < 0 || duration > 60 * 60){ //Max 1 hour.
    logError
      << F("Invalid duration received: ") << duration
      << endl;
    return -1;
  }

  // Check if we still have space.
  if (profile.tail >= PROFILE_SIZE){
    logError
      << F("Cannot append to heating profile. Queue full")
      << endl;
    return -1;
  }

  return 0;
}

void profile_reset() {
  memset(profile.temperature, 0, sizeof(profile.temperature));
  memset(profile.duration, 0, sizeof(profile.duration));
  profile.tail = 0;
  profile.head = 0;
  profile.ramping = false;
  profile.holding = false; //<-- not really required, but reads nicer.
  profile.holdUntil = 0;
  profile.startTime = 0;
  profile.changeTemperature = 0;
  vone->heater.setTargetTemperature(0); //DEFER: refactor to eliminate dependency on vone object
}

int profile_add(const int temperature, const int duration) {
  // Confirm sensible values were received.
  if (s_validate_input(temperature, duration)){
    return -1;
  }

  // Add to temperature and duration to buffer.
  profile.temperature[profile.tail] = temperature;
  profile.duration[profile.tail] = duration;
  profile.tail ++;

  return 0;
}

bool profile_empty() {
  return profile.tail == 0;
}
bool profile_complete() {
  return profile.head >= profile.tail;
}

static float s_computeRampTime(float startTemp, float endTemp) {
  //Heating and cooling rates are not symmetrical (VERY ROUGH ESTIMATE).
  const auto heatingRate = 0.5; // seconds per degrees
  const auto coolingRate = 10.0; // seconds per degrees
  const auto delta = endTemp - startTemp;
  return delta > 0 ? delta * heatingRate : -delta * coolingRate;
}

float profile_sum_durations(int index){
  // Calculates duration given the current index.
  // Starts with index duration and takes account transition ramp (if available)
  float sum = 0;
  for (auto i = index; i < profile.tail; ++i) {
    // add ramp time from previous temp to this one
    if (i > index) {
      sum += s_computeRampTime(profile.temperature[i-1], profile.temperature[i]);
    }

    // Add duration at this temperature
    sum += profile.duration[i];
  }
  return sum;
}

// Calulate how much time is remaining in seconds
float profile_remaining_time(){
  if (profile_empty() || profile_complete()){
    return 0;
  }

  // Iterate through remaining profile and get durations and ramp rates.
  float sum = profile_sum_durations(profile.head);

  if (profile.ramping) {
    // Add our current ramping time.
    sum += s_computeRampTime(vone->heater.currentTemperature(), vone->heater.targetTemperature());
  } else {
    // Substract our elapsed time. (duration - time remaining)
    sum -= profile.duration[profile.head] - (profile.holdUntil - millis()) / 1000ul;
  }

  return sum;
}

void manage_heating_profile() {

  // Early return if our profile is empty
  if (profile_empty()){
    return;
  }

  const unsigned long now = millis();
  const auto target = vone->heater.targetTemperature();
  const auto current = vone->heater.currentTemperature();

  // If we are ramping, check temperature and timeout
  if (profile.ramping) {

    // Check if our safety timeout has been exceeeded. Looks only at time.
    const auto safetyTimeout = minutes(profile.temperature[profile.head] > current ? 8 : 25);
    if (now >= (profile.startTime + safetyTimeout)) {
      logError
        << F("Failed to reach target temperature within timeout period. ")
        << F("Current: ") << current << F("C")
        << F("Target: ") << target << F("C")
        << endl;
      profile_reset();
    }

    // Check if the temperature has not moved at all during heating. Could indicate a loose/fallen thermistor
    // Note: Give the heater 20s to exceed the temp it was at when this step of the profile began.
    //       10s was not enough. This delay gives the heater time to overcome thermal inertia
    //       if it was toggled to an off state (or cooling). Also the changeTemperature may be
    //       higher than expected if we are were on a down-swing when we entered this step. So,
    //       even if we have time to overcome thermal inertia we stil need to climb back to that
    //       point -- that's why 10s was not enough.
    //
    // !!! DO NOT INCREASE PAST 20s !!!
    // If increased to 30s, we may heat the bed over 60deg, while reporting a normal temperature
    // This increased the risk of a user getting hurt. If are tempted to change this value again
    // we should instead change the how we test for this failure.
    //
    // DEFER: it's possible that significant cooling would result in more thermal inertia and a
    //        higher changeTemperature. 20s may not be enough to overcome that.
    if (now >= (profile.startTime + seconds(20)) && (target > current) && abs(current - profile.changeTemperature) < 2) {
      logError
        << F("Temperature change not detected. ")
        << F("Current: ") << current << F("C")
        << F("Target: ") << target << F("C")
        << endl;
      profile_reset();
    }

    // Check if we are within 2 degrees
    if (abs(target - current) < 2) {
      const auto rampDuration = now - profile.startTime;
      const auto averageTempchange = abs(target - profile.changeTemperature) > 2 ? (target - profile.changeTemperature)  / rampDuration : 0;
      log
          << F("Reached ")
          << target
          << F("C from ")
          << profile.changeTemperature
          << F(" in ")
          << rampDuration / 1000.0
          << F("s; average ramp: ")
          << averageTempchange * 1000.0
          << F("C/s. Holding for ")
          << profile.duration[profile.head]
          << F("s")
          << endl;
      profile.holdUntil = now + seconds(profile.duration[profile.head]); // Hold this temp for X seconds
      profile.ramping = false;
      profile.holding = true;
    }

  // If we are holding, have we held on long enough?
  } else if (profile.holding) {

    if (now >= profile.holdUntil) {
      profile.holding = false;
      profile.head ++;

      if(profile_complete()){
        protocol << F("profileComplete") << endl;
        profile_reset();
      }
    }
  } else {
    // If not ramping or holding, set temperature and ramping timeout.
    // Different timeout if we are heating or cooling
    const auto newTarget = profile.temperature[profile.head];
    profile.startTime = now;
    profile.changeTemperature = current; // Take snapshot of current temperature.
    profile.ramping = true;
    vone->heater.setTargetTemperature(newTarget);
  }
}
