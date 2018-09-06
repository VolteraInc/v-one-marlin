#include "../../Marlin.h"
#include "../../temperature_profile.h"
#include "../vone/VOne.h"

static void outputBedTemperatureUpdate(float current, float target, float timeRemaining) {
  protocol
    << F("bedTemperatureUpdate")
    << F(" current:") << FloatWithFormat(current, 1)
    << F(" target:") << FloatWithFormat(target, 1)
    << F(" timeRemaining:") << FloatWithFormat(timeRemaining, 1)
    << endl;
}

void periodic_output() {
  // Run periodically
  static unsigned long nextCheckAt = 0;
  const auto now = millis();
  if (now < nextCheckAt) {
    return;
  }
  nextCheckAt = now + 1000;

  static struct {
    float position[NUM_AXIS];
    struct { float current; float target; } temperature;
  } prev; // Previously reported values

  // Output position on change
  if (memcmp(prev.position, current_position, sizeof(prev.position)) != 0) {
    memcpy(prev.position, current_position, sizeof(prev.position));
    protocol
      << F("positionUpdate")
      << F(" x:") << current_position[X_AXIS]
      << F(" y:") << current_position[Y_AXIS]
      << F(" z:") << current_position[Z_AXIS]
      << F(" e:") << current_position[E_AXIS]
      << endl;
  }

  // Output temperature if temp or target changes or we are running a heating profile
  // NOTE: The temp sensor is noisy so filter small changes.
  const auto current = vone->heater.currentTemperature();
  const auto target = vone->heater.targetTemperature();
  const auto timeRemaining = profile_remaining_time();
  bool tempChanged = abs(prev.temperature.current - current) >= 0.5;
  bool targetChanged = prev.temperature.target != target;
  if ( !profile_empty() || tempChanged || targetChanged) {
      if (targetChanged) {
        // Output an extra data point to signal the change in target temp
        // otherwise you will have a confusing sloped line from the previous
        // temp update to this one.
        // NOTE: sending a 0 for time remaining -- not sure what else to send
        // and this is better than the current time remaining of what may be
        // a different profile.
        outputBedTemperatureUpdate(current, prev.temperature.target, 0);
      }
      outputBedTemperatureUpdate(current, target, timeRemaining);

      prev.temperature.current = current;
      prev.temperature.target = target;
  }

  vone->stepper.periodicReport();
}
