#pragma once

#include "homing/homing.h"
#include "movement/movement.h"
#include "measurement/measurement.h"
#include "probing/probing.h"

class Endstop;
namespace tools {
  class Tool;
  class Probe;
}


// XY positioner
const float defaultXyPositionerCycles = 2;
enum HowToMoveToZ { useConfiguredZ, usePlateBackOffForZ, skipMoveInZ };
int xyPositionerTouch(tools::Tool& tool, const Endstop& endstop, float& measurement);
int xyPositionerFindCenter(tools::Tool& tool, long cycles, float& centerX, float& centerY, enum HowToMoveToZ howToMoveToZ = useConfiguredZ);
int moveToXyPositioner(tools::Tool& tool, enum HowToMoveToZ howToMoveToZ = useConfiguredZ);

// Calibration plate
int measureProbeDisplacement(tools::Probe& probe, float& displacement);

// Bed
const float bedBoundsMinY = 40.0f;
bool isOverBed(float x, float y);
bool haveBedHeightMap();
float bedHeightAt(float x, float y);

// LEDs
int overrideLeds(char r, char b, char g, short pace = 3); // 3 = fast pulse
