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
int prepareToolToMove();
int resetToolPreparations();
void sendToolStatusUpdate();

// Probe
enum ProbeTriggerStates {
  PROBE_UNKNOWN = 0,
  PROBE_OFF = 1,
  PROBE_ON = 2,
  PROBE_TRIGGERED = 3,
};

enum ProbeTriggerStates readProbeTriggerState();
const char* probeTriggerStateAsString(enum ProbeTriggerStates state);
float readProbePinVoltage();
