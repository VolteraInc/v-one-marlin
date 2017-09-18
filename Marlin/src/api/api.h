#pragma once

#include "tools/tools.h"
#include "movement/movement.h"
#include "pins/pins.h"

// Measurement
const auto DefaultMeasureAtSwitchReleaseDelay = 0u;
int measureAtSwitch(int axis, int direction, float maxTravel, float& measurement);
int retractFromSwitch(int axis, int direction, float retractDistance = useDefaultRetractDistance);
int measureAtSwitchRelease(int axis, int direction, unsigned pin, float& releaseStartedAt, float& releaseCompletedAt, unsigned delay_ms = DefaultMeasureAtSwitchReleaseDelay);


// Move before homing
int meshGears();

// Homing
bool homedXY();
int homeXY();
int homeZ(Tool tool);
bool homedZ();
int rawHome(Tool tool, bool homeX = true, bool homeY = true, bool homeZ = true);
int getHomedState(int axis);
void setHomedState(int axis, int value);
void sendHomedStatusUpdate();
int moveToZSwitchXY(Tool tool);

// XY positioner
const float defaultXyPositionerCycles = 2;
enum HowToMoveToZ { useConfiguredZ, usePlateBackOffForZ };
int xyPositionerTouch(Tool tool, int axis, int direction, float& measurement);
int xyPositionerFindCenter(Tool tool, long cycles, float& centerX, float& centerY, enum HowToMoveToZ howToMoveToZ = useConfiguredZ);
const bool skipMoveInZ = true;
int moveToXyPositioner(Tool tool, enum HowToMoveToZ howToMoveToZ = useConfiguredZ);

// Calibration plate
int measureProbeDisplacement(Tool tool, float& displacement);

// Bed
const float bedBoundsMinY = 40.0f;
bool isOverBed(float x, float y);
bool haveBedHeightMap();
float bedHeightAt(float x, float y);

// LEDs
int overrideLeds(char r, char b, char g, short pace = 3); // 3 = fast pulse
