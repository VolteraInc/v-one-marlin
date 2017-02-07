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
Tool getTool();
int prepareToolToMove(Tool tool);
int resetToolPreparations();
void sendToolStatusUpdate();

// Tool States
enum ToolStates {
  TOOL_STATE_UNKNOWN = 0,
  TOOL_STATE_NOT_MOUNTED = 1,
  TOOL_STATE_MOUNTED = 2,
  TOOL_STATE_TRIGGERED = 3,
};
enum ToolStates readToolState(Tool tool);
const char* toolStateAsString(enum ToolStates state);
float readPogoPinVoltage();

// Retract constants
const float NoRetract = -9999.0f;
const float DefaultRetract = 0.2f;

// Probe
int probe(Tool tool, float& measurement, float additionalRetractDistance = DefaultRetract);

// Dispenser
int setDispenseHeight(Tool tool, float height);
float getDispenseHeight(Tool tool);
