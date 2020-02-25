#include "measurement.h"

#include "../../../serial.h"
#include "../../utils/deltaBasedAveraging.h"


int multiMultiTouch(
  const char* context,
  const Endstop& endstop,
  float& result,
  float speed,
  unsigned maxSamples, unsigned maxTouchesPerSample,
  unsigned* o_samplesTaken, unsigned* o_totalTouches
) {
  const auto memoryCheck = 30u;
  if (maxSamples > memoryCheck) {
    logError
      << F("Unable to ") << context << F(", ")
      << F("the requested number of samples ") << maxSamples
      << F(" exceeds allocated space ") << memoryCheck
      << endl;
    return -1;
  }

  auto samplesTaken = 0u;
  auto totalTouches = 0u;
  float samples[maxSamples];
  for (auto i = 0u; i < maxSamples; ++i) {
    auto touchesUsed = 0u;

    // Measure using multiTouch
    if (multiTouch(
      "measure using multi-touch",
      endstop,
      samples[i],
      speed,
      maxTouchesPerSample,
      &touchesUsed
    )) {
      logError
        << F("Unable to ") << context
        << F(", measurement ") << i + 1
        << F(" did not complete")
        << endl;
      return -1;
    };

    // Update counts
    ++samplesTaken;
    totalTouches += touchesUsed;

    // Return early if stable
    const auto epsilon = 0.000001;
    const auto maxDelta = 0.000625 + epsilon;
    const auto minMatches = 3u;
    const auto maxWindowSize = 4u;
    if (trailingStabilityCheck(samples, samplesTaken, maxWindowSize, minMatches, maxDelta, result)) {
      goto SUCCESS;
    }
  }

  // Use average if measurements never meet the criteria of the given function
  result = average(samples, samplesTaken);
  // TODO: restore multi-touch for beta
  // logNotice
  //   << context
  //   << F(", multi-touch measurements did not meet stability target, using average ")
  //   << F("result: ") << result
  //   << F(", samples: ") << serialArray(samples, samplesTaken);
  //   << endl;

SUCCESS:
  if (o_samplesTaken) { *o_samplesTaken = samplesTaken; }
  if (o_totalTouches) { *o_totalTouches = totalTouches; }

  return 0;
}
