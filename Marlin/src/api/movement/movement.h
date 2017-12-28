#pragma once

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


// Set planner position
int setPositionEOnly(float e);
int setPosition(float x, float y, float z, float e);


int retractFromSwitch(int axis, int direction, float retractDistance = useDefaultRetractDistance);


// TODO: use a namespace, e.g. movement::raw movement::internal

const bool skipMovementSafetyCheck = false;

int asyncRawMove(float x, float y, float z, float e, float f, bool confirmMoveIsSafe = true);

// synchronized relative movement (raw means no endstop checks)
int relativeRawMoveXYZ(float x, float y, float z, float speed_in_mm_per_min = useDefaultFeedrate, bool confirmMoveIsSafe = true);
int relativeRawMoveE(float e, float speed_in_mm_per_min = useDefaultFeedrate);
