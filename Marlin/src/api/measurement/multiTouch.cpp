#include "measurement.h"

#include "../../../Marlin.h"

#include "../../utils/deltaBasedAveraging.h"

// static MarlinSerial& s_echoMeasurements(
//   const float approaches[],
//   const float releaseStarts[],
//   const float releaseEnds[],
//   size_t size
// ) {
//   return MYSERIAL
//     << F("approachs: [") << serialArray(approaches, size) << F("]")
//     << F(", releaseStarts: [") << serialArray(releaseStarts, size) << F("]")
//     << F(", releaseEnds: [") << serialArray(releaseEnds, size) << F("]");
// }

int multiTouch(
  const char* context,
  const Endstop& endstop,
  float& result,
  float speed,
  unsigned maxTouches,
  unsigned* o_touchesUsed
) {
  const auto memoryCheck = 50u;
  if (maxTouches > memoryCheck) {
    logError
      << F("Unable to ") << context
      << F(", requested number of touches ") << maxTouches
      << F(" exceeds allocated space ") << memoryCheck
      << endl;
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
      endstop,
      speed, maxDownwardTravel,
      approaches[i], releaseStarts[i], releaseEnds[i]
    )) {
      logError
        << F("Unable to ") << context
        << F(", measurement ") << i + 1
        << F(" did not complete")
        << endl;
      return -1;
    }

    // Update maxDown for next time
    maxDownwardTravel = releaseEnds[i] - approaches[i] + 0.030;

    // Update count
    ++touchesUsed;

    // Return early if measurements are stable
    const auto epsilon = 0.000001;
    const auto maxDelta = 0.000625 + epsilon;
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
  // logNotice
  //   << context
  //   << F(", multi-touch measurements did not meet stability target, using average. ")
  //   << endl;

SUCCESS:
  if (o_touchesUsed) {
    *o_touchesUsed = touchesUsed;
  }
  // TODO: restore multi-touch for beta
  // log
  //   << context << F(" - touches ")
  //   << F("result: ") << result
  //   << F(", touchesUsed: ") << touchesUsed << F(", ")
  //   << s_echoMeasurements(approaches, releaseStarts, releaseEnds, touchesUsed)
  //   << endl;
  return 0;
}
