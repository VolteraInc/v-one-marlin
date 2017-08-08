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

// No tool
namespace NoTool {
  int prepare(Tool tool);
  int unprepare(Tool tool);
}

// Probe
namespace Probe {
  const float DefaultSpeed = 30;
  const float DefaultMaxSamples = 1u; // 30u;
  const float DefaultMaxTouchesPerSample = 1u; // 10u;
  // TODO: restore multi-touch for beta ^

  int probe(
    Tool tool,
    float& measurement,
    float speed = DefaultSpeed,
    float additionalRetractDistance = DefaultRetract,
    unsigned maxSamples = DefaultMaxSamples,
    unsigned maxTouchesPerSample = DefaultMaxTouchesPerSample,
    unsigned* o_samplesTaken = nullptr,
    unsigned* o_touchesUsed = nullptr
  );
  int prepare(Tool tool);
  int unprepare(Tool tool);
  int partiallyPrepare(const char* context, Tool tool);
  float getProbeDisplacement();
  bool isTriggered(float voltage);
}

// Dispenser
namespace Dispenser {
  int prepare(Tool tool);
  int unprepare(Tool tool);
  int setDispenseHeight(Tool tool, float height);
  float getDispenseHeight(Tool tool);
}

// Router
namespace Router {
  // Take 3s to ramp from 0 to max and we want some buffer too
  const unsigned long RampUpDuration = 3000u + 500u;

  int prepare(Tool tool);
  int unprepare(Tool tool);
  int stopRotationIfMounted(Tool tool);
  int stopRotation(Tool tool);
  int setRotationSpeed(Tool tool, unsigned speed);
  float getRotationSpeed(Tool tool);
}
