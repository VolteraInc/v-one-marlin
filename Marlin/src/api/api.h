#pragma once

#include "movement/movement.h"
#include "measurement/measurement.h"
#include "probing/probing.h"

class Endstop;
namespace tools {
  class Tool;
  class Probe;
}

// Homing
bool homedXY();
int homeXY(tools::Tool& tool);
int homeZ(tools::Tool& tool);
bool homedZ();
int rawHome(tools::Tool& tool, bool homeX = true, bool homeY = true, bool homeZ = true);
int getHomedState(int axis);
void setHomedState(int axis, int value);
void sendHomedStatusUpdate();
int moveToZSwitchXY(tools::Tool& tool);

// XY positioner
const float defaultXyPositionerCycles = 2;
enum HowToMoveToZ { useConfiguredZ, usePlateBackOffForZ, skipMoveInZ };
int xyPositionerTouch(const Endstop& endstop, float& measurement);
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
