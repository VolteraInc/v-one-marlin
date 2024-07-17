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
extern const float defaultXyPositionerCycles;
enum HowToMoveToZ { useConfiguredZ, usePlateBackOffForZ, findStrainZ, skipMoveInZ };
int xyPositionerTouch(tools::Tool& tool, const Endstop& endstop, float& measurement);
int xyPositionerFindCenter(tools::Tool& tool, long cycles, float& centerX, float& centerY, enum HowToMoveToZ howToMoveToZ = findStrainZ);
int moveToXyPositioner(tools::Tool& tool, enum HowToMoveToZ howToMoveToZ = findStrainZ);

// Calibration plate
extern const float MinDisplacement;
extern const float MaxDisplacement;
int measureProbeDisplacement(tools::Probe& probe, float& displacement);

// Bed
extern const float bedBoundsMinY;
bool isOverBed(float x, float y);
bool haveBedHeightMap();
float bedHeightAt(float x, float y);

// LEDs
int overrideLeds(char r, char b, char g, short pace = 3); // 3 = fast pulse
