#pragma once

namespace tools {
  class Tool;
  class NullTool;
  class Probe;
}

//-----------------------------------------------
// Burn-in
int burnInSequence(tools::NullTool& noTool, int steps = 5);


//-----------------------------------------------
// Switch calibration

// Back switch separation
int checkBackSwitchSeparation(tools::Tool& tool);

// Switch position calibration
const auto defaultSwitchPositionCalibrationCycles = 2u;
int calibrateSwitchPositions(
  tools::Probe& probe,
  unsigned cycles = defaultSwitchPositionCalibrationCycles,
  bool storeResults = true
);

// Extent checking
namespace CheckExtents {
  const float useDefaultTolerance = -1;
}
int checkExtents(tools::Tool& tool, float tolerance = CheckExtents::useDefaultTolerance);


//-----------------------------------------------
// Runners
int runCalibrateSwitches(
  tools::Probe& probe,
  unsigned cycles = defaultSwitchPositionCalibrationCycles
);

int runBurnInSequence(tools::NullTool& noTool);
