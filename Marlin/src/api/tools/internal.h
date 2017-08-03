#pragma once

#include "tools.h"

int ensureHomedInXY();
int centerTool(Tool tool);
int retractToolConditionally(float distance, float additionalRetractDistance);
