#include "../api.h"
#include "../../../Marlin.h"
#include "../../vone/endstops/Endstop.h"

int touch(
  const Endstop& endstop,
  float speed,
  float maxTravel,
  float& approach,
  float& releaseStart,
  float& releaseEnd
) {
  const auto startTime = millis();

  if (moveToEndstop(endstop, speed, maxTravel)) {
    return -1;
  }

  approach = current_position[endstop.axis];

  if (measureAtSwitchRelease(endstop, releaseStart, releaseEnd)) {
    return -1;
  }

  if (logging_enabled) {
    const auto duration = millis() - startTime;
    log << F("touch duration: ") << duration << endl;
  }

  return 0;
}

int fastTouch(const Endstop& endstop) {
  const auto startTime = millis();
  float approach, releaseStart, releaseEnd;
  if (touch(
    endstop,
    useDefaultFeedrate, useDefaultMaxTravel,
    approach, releaseStart, releaseEnd
  )) {
    return -1;
  }

  if (logging_enabled) {
    const auto duration = millis() - startTime;
    log
      << F("fastTouch ") << endstop.name
      << F("approach: ") << approach
      << F(", releaseStart: ") << releaseStart
      << F(", releaseEnd: ") << releaseEnd
      << F(", duration: ") << duration
      << endl;
  }

  return 0;
}
