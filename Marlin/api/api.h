#pragma once

#include "./tools/tools.h"

// Movement
const int useDefaultFeedrate = -1;
const float useDefaultMaxTravel = 9999.9f;
const bool ignoreDispenseHeight = false;
const float useDefaultRetractDistance = -1.0f;
float getDefaultFeedrate();

int outputMovementStatus();

// Absolute movement
int move(Tool tool, float x, float y, float z, float e, float f, bool applyDispenseHeight = ignoreDispenseHeight);
int moveXY(Tool tool, float x, float y, float f = useDefaultFeedrate);
int moveZ(Tool tool, float z, float f = useDefaultFeedrate, bool applyDispenseHeight = ignoreDispenseHeight);

// Relative movement
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

// Homing
int home(Tool tool, bool homeX = true, bool homeY = true, bool homeZ = true);
bool homedXY();
int homeXY();
int homeZ(Tool tool);
bool homedZ();
int getHomedState(int axis);
void setHomedState(int axis, int value);
void sendHomedStatusUpdate();
int moveToZSwitch(Tool tool);

// XY positioner
const float defaultXyPositionerCycles = 2;
int xyPositionerTouch(Tool tool, int axis, int direction, float& measurement);
int xyPositionerFindCenter(Tool tool, long cycles, float& centerX, float& centerY);
const bool skipMoveInZ = true;
int moveToXyPositioner(Tool tool, bool skipMoveInZ = false);
int calibrateKeyPositions(Tool tool, long cycles = defaultXyPositionerCycles);

// Calibration plate
int measureProbeDisplacement(Tool tool, float& displacement);
