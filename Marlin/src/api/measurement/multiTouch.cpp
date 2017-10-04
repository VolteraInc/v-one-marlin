#include "measurement.h"

#include "../../../Marlin.h"

#include "../../utils/deltaBasedAveraging.h"


static void s_echoMeasurements(
  const float approaches[],
  const float releaseStarts[],
  const float releaseEnds[],
  size_t size
) {
  SERIAL_ECHOPGM("approachs: ["), serialArray(approaches, size); SERIAL_ECHOPGM("]");
  SERIAL_ECHOPGM(", releaseStarts: ["), serialArray(releaseStarts, size); SERIAL_ECHOPGM("]");
  SERIAL_ECHOPGM(", releaseEnds: ["), serialArray(releaseEnds, size); SERIAL_ECHOPGM("]");
}


int multiTouch(
  const char* context,
  float& result,
  float speed,
  unsigned maxTouches,
  unsigned* o_touchesUsed
) {
  const auto memoryCheck = 50u;
  if (maxTouches > memoryCheck) {
    SERIAL_ERROR_START;
    SERIAL_ERROR("Unable to "); SERIAL_ERROR(context);
    SERIAL_PAIR(", requested number of touches ", maxTouches);
    SERIAL_PAIR(" exceeds allocated space ", memoryCheck);
    SERIAL_EOL;
    return -1;
  }

  auto touchesUsed = 0u;
  float approaches[maxTouches];
  float releaseStarts[maxTouches];
  float releaseEnds[maxTouches];
  float maxDownwardTravel = 1; // TODO better initial value ? maybe use maxTravel from measureAtSwitchRelease?
  for (auto i = 0u; i < maxTouches; ++i) {
    // Measure
    if (touch(
      Z_AXIS, -1,
      speed, maxDownwardTravel,
      approaches[i], releaseStarts[i], releaseEnds[i]
    )) {
      SERIAL_ERROR_START;
      SERIAL_ERROR("Unable to "); SERIAL_ERROR(context);
      SERIAL_PAIR(", measurement ", i + 1);
      SERIAL_ERROR(" did not complete");
      SERIAL_EOL;
      return -1;
    }

    // Update maxDown for next time
    maxDownwardTravel = releaseEnds[i] - approaches[i] + 0.030;

    // Update count
    ++touchesUsed;

    // Return early if measurements are stable
    const auto maxDelta = 0.000625f;
    const auto minMatches = 4u;
    const auto maxWindowSize = 6u;
    if (trailingStabilityCheck(releaseStarts, touchesUsed, maxWindowSize, minMatches, maxDelta, result)) {
      goto SUCCESS;
    }
  }

  // Use average if measurements never meet the stability criteria
  // TODO: Watch for this warning, examine the data to see if there's
  // something smarter than an average. I don't want to attempt that
  // right now without good data. Once i have good data i can start
  // returning an error, if warranted.
  result = average(releaseStarts, touchesUsed);
  // TODO: restore multi-touch for beta
  // SERIAL_ECHO_START;
  // SERIAL_ECHOPGM("Notice: ("); SERIAL_ECHO(context);
  // SERIAL_ECHOPGM(") multi-touch measurements did not meet stability target, using average. ");
  // SERIAL_EOL;

SUCCESS:
  if (o_touchesUsed) {
    *o_touchesUsed = touchesUsed;
  }
  // TODO: restore multi-touch for beta
  // SERIAL_ECHO_START;
  // SERIAL_ECHO(context);
  // SERIAL_ECHOPGM(" - touches ");
  // SERIAL_PAIR("result: ", result);
  // SERIAL_PAIR(", touchesUsed: ", touchesUsed);
  // SERIAL_ECHOPGM(", "); s_echoMeasurements(approaches, releaseStarts, releaseEnds, touchesUsed);
  // SERIAL_EOL;
  return 0;
}
