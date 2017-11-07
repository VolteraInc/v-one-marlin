#include "Marlin.h"
#include "temperature.h"

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
} profile; // Previously reported values


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
  setTargetBed(0);
}

void profile_add(const int temperature, const int duration) {
  //Add to temperature and duration to buffer.
  profile.temperature[profile.tail] = temperature;
  profile.duration[profile.tail] = duration;
  profile.tail ++;
}

bool profile_empty(){
  return profile.tail == 0;
}
bool profile_complete() {
  return profile.head >= profile.tail;
}

//Heating and cooling rates are not symmetrical (VERY ROUGH ESTIMATE).
#define HEAT_RATE (0.5) // [seconds / degrees]
#define COOL_RATE (10)  // [seconds / degrees]

float profile_sum_durations(int index){
  // Calculates duration given the current index.
  // Starts with index duration and takes account transition ramp (if available)
  float delta = 0;
  float sum = 0;

  while(index < profile.tail){
    sum += profile.duration[index];

    // Peek ahead and add ramp rate.
    if (index + 1 < profile.tail){
      delta = profile.temperature[index+1] - profile.temperature[index];
      sum += delta > 0 ? delta * HEAT_RATE : -delta * COOL_RATE;
    }
    index ++;
  }
  return sum;
}

float profile_remaining_time(){
  // Calulate how much time is remaining in seconds (very rough estimate of ramping and cooling rates)
  float sum;
  float delta;

  if (profile_empty() || profile_complete()){
    return 0;
  }

  // Iterate through remaining profile and get durations and ramp rates.
  sum = profile_sum_durations(profile.head);

  if (profile.ramping){
    // Add our current ramping time.
    delta = target_temperature_bed - current_temperature_bed;
    sum += delta > 0 ? delta * HEAT_RATE : -delta * COOL_RATE;
  }

  else {
    // Substract our elapsed time. (duration - time remaining)
    sum -= profile.duration[profile.head] - (profile.holdUntil - millis())/1000;
  }

  return sum;
}

void manage_heating_profile() {

  // Early return if our profile is empty
  if (profile_empty()){
    return;
  }

  const unsigned long now = millis();

  // If we are ramping, check temperature and timeout
  if (profile.ramping) {

    // Check if our safety timeout has been exceeeded. Looks only at time.
    if (now >= profile.safetyTimeout) {
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Failed to reach target temperature within timeout period. Current: "); SERIAL_ERROR(current_temperature_bed);
      SERIAL_ERRORPGM("C Target: "); SERIAL_ERROR(target_temperature_bed);
      SERIAL_ERRORPGM("C\n");
      profile_reset();
    }

    // Check if the temperature has not moved at all during heating. Could indicate a loose/fallen thermistor
    if (now >= profile.changeTimeout && (target_temperature_bed > current_temperature_bed) && abs(current_temperature_bed - profile.changeTemperature) < 2) {
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Temperature change not detected. Current: "); SERIAL_ERROR(current_temperature_bed);
      SERIAL_ERRORPGM("C Target: "); SERIAL_ERROR(target_temperature_bed);
      SERIAL_ERRORPGM("C\n");
      profile_reset();
    }

    // Check if we are within 2 degrees
    if (abs(target_temperature_bed - current_temperature_bed) < 2) {
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("Reached target temperature. Holding for "); SERIAL_ECHO(profile.duration[profile.head]);
      SERIAL_ECHOPGM(" seconds");
      SERIAL_ECHOPGM("\n");
      profile.holdUntil = now + profile.duration[profile.head] * 1000; // Hold this temp for X seconds
      profile.ramping = false;
      profile.holding = true;
    }
    return;
  }

  // If we are holding, have we held on long enough?
  if (profile.holding) {

    if (now >= profile.holdUntil) {
      profile.holding = false;
      profile.head ++;

      if(profile_complete()){
        SERIAL_PROTOCOLLNPGM("profileComplete");
        profile_reset();
      }
    }
    return;
  }

  // If not ramping or holding, set temperature and ramping timeout. Different timeout if we are heating or cooling
  unsigned long timeout = profile.temperature[profile.head] > current_temperature_bed ? (unsigned long)4*60*1000 : (unsigned long)25*60*1000;
  profile.safetyTimeout = now + timeout;
  profile.changeTimeout = now + (unsigned long)10*1000; // 10 seconds to register a change in temperature.
  profile.changeTemperature = current_temperature_bed; // Take snapshot of current temperature.
  profile.ramping = true;
  setTargetBed(profile.temperature[profile.head]);
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("New target Temperature: "); SERIAL_ECHOLN(profile.temperature[profile.head]);
}
