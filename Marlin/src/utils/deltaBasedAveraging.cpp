#include "deltaBasedAveraging.h"

#include <Arduino.h>

// Computing the average by averaging the deltas is less
// susceptable to loss of precision. i.e. Sum will likely
// exceed 100, maybe 1000, leaving little room for decimals.
// The sum of deltas does not have this problem.
// For Reference:
//   avg = (A + B + C) / 3
//       = (A + B + C) / 3 - C + C
//       = (A + B + C - 3C) / 3 + C
//       = (A-C + B-C + 0) / 3 + C
//       = (A-C + B-C) / 3 + C

float average(const float data[], size_t size) {
  auto dsum = 0.0f;
  const auto& last = data[size - 1];
  for (auto i = 0u; i < size; ++i) {
    dsum += data[i] - last;
  }
  return last + dsum / size;
}

float filteredAverage(const float data[], size_t size, float maxDelta, unsigned* o_numMatches) {
  auto dsum = 0.0f;
  const auto& last = data[size - 1];
  auto count = 0;
  for (auto i = 0u; i < size; ++i) {
    const auto delta = data[i] - last;
    if (abs(delta) <= maxDelta) {
      ++count;
      dsum += delta;
    }
  }

  if (o_numMatches) { *o_numMatches = count; }

  auto result = last + dsum / count;
  return result;
}
