#pragma once

// Movement
const int useDefaultFeedrate = -1;
int outputMovementStatus();
float getDefaultFeedrate();
int move(float x, float y, float z, float e, float f);
int moveXY(float x, float y, float f = useDefaultFeedrate);
int relativeMove(float x, float y, float z, float e, float f);
int moveToLimit(int axis, int direction, float speed_in_mm_per_min, float maxTravel = 9999.9f);
int probe(float retractDistance, float& measurement);
