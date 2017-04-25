#pragma once

int prepareProbe(Tool tool);
int prepareDispenser(Tool tool);
int prepareRouter(Tool tool);

int confirmRequiredToolAttached(const char* context, Tool tool, Tool requiredTool);
int confirmMountedAndNotTriggered(const char* context, Tool tool);
int ensureHomedInXY();
int ensureHomedInZ(Tool tool);
int centerTool(Tool tool);
int lowerUntilToolContacts(Tool tool);
int retractToolConditionally(float distance, float additionalRetractDistance);
