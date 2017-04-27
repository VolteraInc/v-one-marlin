#pragma once

#include "tools.h"

int prepareProbe(Tool tool);
int prepareDispenser(Tool tool);
int prepareRouter(Tool tool);

int confirmMountedAndNotTriggered(const char* context, Tool tool, Tool requiredTool);
int ensureHomedInXY();
int ensureHomedInZ(Tool tool);
int centerTool(Tool tool);
int lowerUntilToolContacts(Tool tool);
int retractToolConditionally(float distance, float additionalRetractDistance);
