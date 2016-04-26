#pragma once

// Movement
int outputMovementStatus();
int move(float x, float y, float z, float e, float f);
int relativeMove(float x, float y, float z, float e, float f);
int moveToLimit(int axis, int direction, float speed_in_mm_per_min, float maxTravel = 9999.9f);
int probe(float retractDistance, float& measurement);
