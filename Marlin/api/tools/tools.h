#pragma once

// Tools
enum Tool {
  TOOLS_NONE = 0,
  TOOLS_PROBE = 1,
  TOOLS_DISPENSER = 2,
};

const char* toolTypeAsString(Tool tool);
int outputToolStatus();
void setTool(Tool tool);
int prepareToolToMove();
int resetToolPreparations();
