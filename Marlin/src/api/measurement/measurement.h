#pragma once

#include "../../../MarlinConfig.h"
#include "../../../Axis.h"

class Endstop;

int touch(
  const Endstop& endstop,
  float speed, float maxTravel,
  float& approach, float& releaseStart, float& releaseEnd
);

int fastTouch(const Endstop& endstop);

int multiTouch(
  const char* context,
  const Endstop& endstop,
  float& measurement,
  float speed,
  unsigned numTouches,
  unsigned* o_touchesUsed = nullptr
);

int multiMultiTouch(
  const char* context,
  const Endstop& endstop,
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

const auto DefaultMeasureAtSwitchReleaseDelay = 0u;
int measureAtSwitch(const Endstop& endstop, float maxTravel, float& measurement);
int measureAtSwitchRelease(const Endstop& endstop, float& releaseStartedAt, float& releaseCompletedAt, unsigned delay_ms = DefaultMeasureAtSwitchReleaseDelay);
