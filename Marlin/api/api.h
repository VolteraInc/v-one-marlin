#pragma once

#include "./tools/tools.h"

// Movement
const int useDefaultFeedrate = -1;
const float useDefaultMaxTravel = 9999.9f;
int outputMovementStatus();
float getDefaultFeedrate();

// Absolute movement
int move(float x, float y, float z, float e, float f);
int moveXY(float x, float y, float f = useDefaultFeedrate);
int moveZ(float z, float f = useDefaultFeedrate);

// Relative movement
int relativeMove(float x, float y, float z, float e, float f = useDefaultFeedrate);

// Move until switch hit
int moveToLimit(int axis, int direction, float f = useDefaultFeedrate, float maxTravel = useDefaultMaxTravel);
int raise();

// Measurement
int measureAtSwitch(int axis, int direction, float maxTravel, float& measurement);
int retractFromSwitch(int axis, int direction);
int probe(float& measurement);

// Set planner position
int setPositionEOnly(float e);
int setPosition(float x, float y, float z, float e);

// Homing
int home(bool homeX = true, bool homeY = true, bool homeZ = true);
bool homedXY();
int homeXY();
int homeZ();
bool homedZ();
int getHomedState(int axis);
void setHomedState(int axis, int value);
void sendHomedStatusUpdate();
int moveToZSwitch();

// XY positioner
const float defaultXyPositionerCycles = 2;
int xyPositionerTouch(int axis, int direction, float& measurement);
int xyPositionerFindCenter(long cycles, float& centerX, float& centerY);
int moveToXyPositioner();

// Calibration plate
int measureProbeDisplacement(float& displacement);
