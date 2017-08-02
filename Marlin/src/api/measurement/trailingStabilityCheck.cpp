#include "../../utils/deltaBasedAveraging.h"

// Return early if stable
bool trailingStabilityCheck(const float data[], size_t size, unsigned maxWindowSize, unsigned minMatches, float maxDelta, float& result) {
  const auto windowSize = min(maxWindowSize, size);
  if (windowSize < minMatches) {
    return false;
  }

  const auto& windowStart = &data[size - windowSize];
  unsigned numMatches;
  const auto average = filteredAverage(windowStart, windowSize, maxDelta, &numMatches);

  if (numMatches < minMatches) {
    return false;
  } else {
    result = average;
    return true;
  }
}
