#pragma once

namespace tools {
  class Probe;
}

class Point2d;
class Point3d;

namespace probing {

  // ----------------------------------------------
  // Probe Hole

  namespace ProbeHole {
    const auto DefaultMinPoints = 3u;
    const auto DefaultProbePitch = 1.0;
  }

  int probeHole(
    tools::Probe& probe,
    const Point2d& center,
    float radius,
    Point3d measurements[],
    unsigned int maxMeasurements,
    unsigned int* o_numMeasurements = nullptr,
    unsigned int minPoints = ProbeHole::DefaultMinPoints,
    float probePitch = ProbeHole::DefaultProbePitch
  );

} // namespace probing
