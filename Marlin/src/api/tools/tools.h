#pragma once

// Tools
enum Tool {
  TOOLS_NONE = 0,
  TOOLS_PROBE = 1,
  TOOLS_DISPENSER = 2,
  TOOLS_ROUTER = 3,
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
  TOOL_STATE_TRIGGERED = 2,
  TOOL_STATE_PROBE_MOUNTED = 3,
  TOOL_STATE_ROUTER_MOUNTED = 4
};
enum ToolStates classifyVoltage(Tool tool, float voltage);
enum ToolStates determineToolState(Tool tool);
const char* toolStateAsString(enum ToolStates state);

// Retract constants
const float NoRetract = -9999.0f;
const float DefaultRetract = 0.2f;

// Tool functions
int confirmMountedAndNotTriggered(const char* context, Tool tool, Tool requiredTool);

// Probe
int probe(Tool tool, float& measurement, float speed, float additionalRetractDistance = DefaultRetract);

// Dispenser
int setDispenseHeight(Tool tool, float height);
float getDispenseHeight(Tool tool);

// Router
const unsigned long RouterRampUpDuration = 3000u + 500u; //Take 3s to ramp from 0 to max and we want some buffer too
int setRotationSpeed(Tool tool, int speed);
float getRotationSpeed(Tool tool);
