#include "probing.h"
#include "../movement/movement.h"
#include "../../vone/tools/Probe.h"
#include "../../../serial.h"

int probing::probeHole(
  tools::Probe& probe,
  const Point2d& center,
  float radius,
  Point3d measurements[],
  unsigned int maxMeasurements,
  unsigned int* o_numMeasurements,
  unsigned int minPoints,
  float probePitch
) {
  // Determine number of measurements
  const auto circumference = TWO_PI * radius;
  const unsigned int numMeasurements = max(
    minPoints,
    ceil(circumference/probePitch)
  );

  // Confirm we have enough room for the measurements
  if (numMeasurements > maxMeasurements) {
    SERIAL_ERROR_START;
    SERIAL_PAIR("Unable to probe hole, required number of measurements (", numMeasurements);
    SERIAL_PAIR(") exceeds maximum, ", maxMeasurements);
    SERIAL_EOL;
    return -1;
  }

  // Probe around the hole
  // Note: we start at the top of the hole
  Point2d start = center.offset(0, -radius - 0.200);
  const auto thetaPerSegment = TWO_PI / numMeasurements;
  for (auto idx = 0u; idx < numMeasurements; ++idx) {
    const auto dest = start.rotate(idx * thetaPerSegment, center);
    measurements[idx].x = dest.x;
    measurements[idx].y = dest.y;
    if (
      probe.moveToSafeHeight() ||
      moveXY(probe, dest.x, dest.y) ||
      probe.probe(measurements[idx].z)
    ) {
      return -1;
    }
  }

  if (o_numMeasurements) {
    *o_numMeasurements = numMeasurements;
  }

  return 0;
}
