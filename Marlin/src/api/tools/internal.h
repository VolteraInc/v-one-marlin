#pragma once

struct Point2D {
  float x = 0.0f;
  float y = 0.0f;
};

int prepareProbe(Tool tool);
int prepareDispenser(Tool tool);
bool probeMounted();
int confirmRequiredToolAttached(const char* context, Tool tool, Tool requiredTool);
int confirmMountedAndNotTriggered(const char* context, Tool tool);
int ensureHomedInXY();
int ensureHomedInZ(Tool tool);
int centerTool(Tool tool);
int lowerUntilToolContacts(Tool tool);
int retractToolConditionally(float distance, float additionalRetractDistance);
