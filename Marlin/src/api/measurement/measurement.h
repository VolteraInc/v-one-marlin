#pragma once

#include "Arduino.h"

int touch(
  int axis, int direction, int pin,
  float speed, float maxTravel,
  float& approach, float& releaseStart, float& releaseEnd
);

int fastTouch();

int multiTouch(
  const char* context,
  float& measurement,
  float speed,
  unsigned numTouches,
  unsigned* o_touchesUsed = nullptr
);

int multiMultiTouch(
  const char* context,
  float& result,
  float speed,
  unsigned maxSamples, unsigned maxTouchesPerSample,
  unsigned* o_samplesTaken = nullptr, unsigned* o_totalTouches = nullptr
);

bool trailingStabilityCheck(
  const float data[],
  size_t size,
  unsigned maxWindowSize,
  unsigned minMatches,
  float maxDelta,
  float& result
);
