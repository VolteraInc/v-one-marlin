#pragma once

const bool skipMovementSafetyCheck = false;

int asyncRawMove(float x, float y, float z, float e, float f, bool confirmMoveIsSafe = true);
