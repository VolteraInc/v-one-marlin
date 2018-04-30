#include "../api.h"
#include "../../../Marlin.h"

int touch(
  int axis,
  int direction,
  float speed,
  float maxTravel,
  float& approach,
  float& releaseStart,
  float& releaseEnd
) {
  const auto startTime = millis();

  if (moveToLimit(axis, direction, speed, maxTravel)) {
    return -1;
  }

  approach = current_position[axis];

  if (measureAtSwitchRelease(axis, direction, releaseStart, releaseEnd)) {
    return -1;
  }

  if (logging_enabled) {
    const auto duration = millis() - startTime;
    log << F("touch duration: ") << duration << endl;
  }

  return 0;
}

int fastTouch() {
  const auto startTime = millis();
  float approach, releaseStart, releaseEnd;
  if (touch(
    Z_AXIS, -1,
    useDefaultFeedrate, useDefaultMaxTravel,
    approach, releaseStart, releaseEnd
  )) {
    return -1;
  }

  if (logging_enabled) {
    const auto duration = millis() - startTime;
    log
      << F("fastTouch ")
      << F("approach: ") << approach
      << F(", releaseStart: ") << releaseStart
      << F(", releaseEnd: ") << releaseEnd
      << F(", duration: ") << duration
      << endl;
  }

  return 0;
}
