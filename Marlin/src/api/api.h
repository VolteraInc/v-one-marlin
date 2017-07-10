#pragma once

#include "./tools/tools.h"

// Movement
const int useDefaultFeedrate = -1;
const float useDefaultMaxTravel = 9999.9f;
const bool ignoreDispenseHeight = false;
const float useDefaultRetractDistance = -1.0f;
float getDefaultFeedrate();

int outputMovementStatus();

// Enqueue movements (do not wait for movement to finish)
int asyncMove(Tool tool, float x, float y, float z, float e, float f, bool applyDispenseHeight = ignoreDispenseHeight);
int asyncRelativeMove(Tool tool, float x, float y, float z, float e, float f = useDefaultFeedrate);

// Absolute movement (synchronized)
int moveXY(Tool tool, float x, float y, float f = useDefaultFeedrate);
int moveZ(Tool tool, float z, float f = useDefaultFeedrate, bool applyDispenseHeight = ignoreDispenseHeight);

// Relative movement (synchronized)
int relativeMove(Tool tool, float x, float y, float z, float e, float f = useDefaultFeedrate);

// Move until switch hit
int moveToLimit(int axis, int direction, float f = useDefaultFeedrate, float maxTravel = useDefaultMaxTravel);
int raise();

// Measurement
int measureAtSwitch(int axis, int direction, float maxTravel, float& measurement);
int retractFromSwitch(int axis, int direction, float retractDistance = useDefaultRetractDistance);

// Set planner position
int setPositionEOnly(float e);
int setPosition(float x, float y, float z, float e);

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
int calibrateKeyPositions(Tool tool, long cycles = defaultXyPositionerCycles);

// Calibration plate
int measureProbeDisplacement(Tool tool, float& displacement);

// Bed
const float bedBoundsMinY = 40.0f;
bool isOverBed(float x, float y);
bool haveBedHeightMap();
float bedHeightAt(float x, float y);

// LEDs
int overrideLeds(char r, char b, char g, short pace = 3); // 3 = fast pulse

// burn-in
int runBurnInSequence(Tool tool, int steps = 5);
