#pragma once

namespace tools {
  class NullTool;
  class Probe;
}

// ----------------------------------------------
// Burn-in
int burnInSequence(tools::NullTool& noTool, int steps = 5);
int runBurnInSequence(tools::NullTool& noTool);

// ----------------------------------------------
// Switch calibration

int checkBackSwitchSeparation(tools::Probe& probe);

const auto defaultSwitchPositionCalibrationCycles = 2u;
int calibrateSwitchPositions(
  tools::Probe& probe,
  unsigned cycles = defaultSwitchPositionCalibrationCycles,
  bool storeResults = true
);

int runCalibrateSwitches(
  tools::Probe& probe,
  unsigned cycles = defaultSwitchPositionCalibrationCycles
);
