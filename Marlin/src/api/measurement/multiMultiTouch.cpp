#include "measurement.h"

#include "../../../Marlin.h"
#include "../../utils/deltaBasedAveraging.h"


int multiMultiTouch(
  const char* context,
  float& result,
  float speed,
  unsigned maxSamples, unsigned maxTouchesPerSample,
  unsigned* o_samplesTaken, unsigned* o_totalTouches
) {
  const auto memoryCheck = 30u;
  if (maxSamples > memoryCheck) {
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to "); SERIAL_ERROR(context);
    SERIAL_PAIR(", requested number of samples ", maxSamples);
    SERIAL_PAIR(" exceeds allocated space ", memoryCheck);
    SERIAL_EOL;
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
      samples[i],
      speed,
      maxTouchesPerSample,
      &touchesUsed
    )) {
      SERIAL_ERROR_START;
      SERIAL_ERROR("Unable to "); SERIAL_ERROR(context);
      SERIAL_PAIR(", measurement ", i + 1);
      SERIAL_ERROR(" did not complete");
      SERIAL_EOL;
      return -1;
    };

    // Update counts
    ++samplesTaken;
    totalTouches += touchesUsed;

    // Return early if stable
    const auto maxDelta = 0.000625f;
    const auto minMatches = 3u;
    const auto maxWindowSize = 4u;
    if (trailingStabilityCheck(samples, samplesTaken, maxWindowSize, minMatches, maxDelta, result)) {
      goto SUCCESS;
    }
  }

  // Use average if measurements never meet the criteria of the given function
  result = average(samples, samplesTaken);
  // TODO: restore multi-touch for beta
  // SERIAL_ECHO_START;
  // SERIAL_ECHOPGM("Notice: ("); SERIAL_ECHO(context);
  // SERIAL_ECHO(") multi-touch measurements did not meet stability target, using average ");
  // SERIAL_PAIR("result: ", result);
  // SERIAL_ECHOPGM(", samples: "); serialArray(samples, samplesTaken);
  // SERIAL_EOL;

SUCCESS:
  if (o_samplesTaken) { *o_samplesTaken = samplesTaken; }
  if (o_totalTouches) { *o_totalTouches = totalTouches; }

  return 0;
}
