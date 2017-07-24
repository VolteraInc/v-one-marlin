#pragma once

#include "../tools/tools.h"


// Burn-in
int burnInSequence(Tool tool, int steps = 5);
int runBurnInSequence(Tool tool);


// Switch position calibration
const auto defaultSwitchPositionCalibrationCycles = 2u;
int calibrateSwitchPositions(
  Tool tool,
  unsigned cycles = defaultSwitchPositionCalibrationCycles,
  bool storeResults = true
);
int runCalibrateSwitchPositions(
  Tool tool,
  unsigned cycles = defaultSwitchPositionCalibrationCycles
);
