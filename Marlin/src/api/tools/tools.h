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

// Probe
enum ProbeTriggerStates {
  PROBE_UNKNOWN = 0,
  PROBE_OFF = 1,
  PROBE_ON = 2,
  PROBE_TRIGGERED = 3,
};

const float NoRetract = -9999.0f;
const float DefaultRetract = 0.2f;
int probe(Tool tool, float& measurement, float additionalRetractDistance = DefaultRetract);
enum ProbeTriggerStates readProbeTriggerState();
const char* probeTriggerStateAsString(enum ProbeTriggerStates state);
float readProbePinVoltage();

// Dispenser
int setDispenseHeight(Tool tool, float height);
float getDispenseHeight(Tool tool);

// Drill
void drill_enable();
void drill_disable();
void drill_set_speed(int new_feedrate);
float drill_set_frequency(float new_frequency);
void drill_monitor();
