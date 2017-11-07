#include "Marlin.h"
#include "src/vone/VOne.h"
#include "src/vone/bed/heater/Heater.h"

#define PROFILE_SIZE (10)
static struct {
  unsigned long holdUntil = 0;
  unsigned long safetyTimeout = 0;
  unsigned long changeTimeout = 0;
  float changeTemperature = 0;
  bool ramping = false;
  bool holding = false;
  int head = 0; // Indicates start of queue.
  int tail = 0; // Indicates end of queue
  int temperature[PROFILE_SIZE]; // Temperature in degrees C
  unsigned long duration[PROFILE_SIZE]; // Duration in seconds
} profile;


int profile_validate_input(const int temperature, const int duration){

  // Ensure both parameters were received.
  if (temperature == 0 || duration == 0){
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Cannot interpret heating profile. Temperature: "); SERIAL_ERROR(temperature);
    SERIAL_ERRORPGM(" Duration: "); SERIAL_ERROR(duration);
    SERIAL_ERRORPGM("\n");
    return -1;
  }

  if (temperature < 0 || temperature > 240){
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Invalid temperature target received: "); SERIAL_ERROR(temperature);
    SERIAL_ERRORPGM("\n");
    return -1;
  }

  if (duration < 0 || duration > 60 * 60){ //Max 1 hour.
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Invalid duration time received: "); SERIAL_ERROR(duration);
    SERIAL_ERRORPGM("\n");
    return -1;
  }

  // Check if we still have space.
  if (profile.tail >= PROFILE_SIZE){
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Cannot append to heating profile. Queue full");
    SERIAL_ERRORPGM("\n");
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
  profile.safetyTimeout = 0;
  profile.changeTimeout = 0;
  profile.changeTemperature = 0;
  vone->heater.setTargetTemperature(0); //DEFER: refactor to eliminate dependency on vone object
}

void profile_add(const int temperature, const int duration) {
  // Add to temperature and duration to buffer.
  profile.temperature[profile.tail] = temperature;
  profile.duration[profile.tail] = duration;
  profile.tail ++;
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
    sum -= profile.duration[profile.head] - (profile.holdUntil - millis()) / 1000;
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
    if (now >= profile.safetyTimeout) {
      SERIAL_ERROR_START;
      SERIAL_PAIR("Failed to reach target temperature within timeout period. Current: ", current);
      SERIAL_PAIR("C Target: ", target);
      SERIAL_ERRORPGM("C");
      SERIAL_EOL;
      profile_reset();
    }

    // Check if the temperature has not moved at all during heating. Could indicate a loose/fallen thermistor
    if (now >= profile.changeTimeout && (target > current) && abs(current - profile.changeTemperature) < 2) {
      SERIAL_ERROR_START;
      SERIAL_PAIR("Temperature change not detected. Current: ", current);
      SERIAL_PAIR("C Target: ", target);
      SERIAL_ERRORPGM("C");
      SERIAL_EOL;
      profile_reset();
    }

    // Check if we are within 2 degrees
    if (abs(target - current) < 2) {
      SERIAL_ECHO_START;
      SERIAL_PAIR("Reached target temperature. Holding for ", profile.duration[profile.head]);
      SERIAL_ECHOPGM(" seconds");
      SERIAL_EOL;
      profile.holdUntil = now + profile.duration[profile.head] * 1000; // Hold this temp for X seconds
      profile.ramping = false;
      profile.holding = true;
    }

  // If we are holding, have we held on long enough?
  } else if (profile.holding) {

    if (now >= profile.holdUntil) {
      profile.holding = false;
      profile.head ++;

      if(profile_complete()){
        SERIAL_PROTOCOLLNPGM("profileComplete");
        profile_reset();
      }
    }
  } else {
    // If not ramping or holding, set temperature and ramping timeout.
    // Different timeout if we are heating or cooling
    const auto newTarget = profile.temperature[profile.head];
    const auto safetyTimeout = newTarget > current ? 4 * 60 * 1000ul : 25 * 60 * 1000ul;
    profile.safetyTimeout = now + safetyTimeout;
    profile.changeTimeout = now + 10 * 1000ul; // 10 seconds to register a change in temperature.
    profile.changeTemperature = current; // Take snapshot of current temperature.
    profile.ramping = true;
    vone->heater.setTargetTemperature(newTarget);
  }
}
