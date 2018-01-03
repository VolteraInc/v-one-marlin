#pragma once

namespace tools {
  class NullTool;
  class Probe;
}

// Burn-in
int burnInSequence(tools::NullTool& noTool, int steps = 5);
int runBurnInSequence(tools::NullTool& noTool);


// Switch position calibration
const auto defaultSwitchPositionCalibrationCycles = 2u;
int calibrateSwitchPositions(
  tools::Probe& probe,
  unsigned cycles = defaultSwitchPositionCalibrationCycles,
  bool storeResults = true
);
int runCalibrateSwitchPositions(
  tools::Probe& probe,
  unsigned cycles = defaultSwitchPositionCalibrationCycles
);
