#pragma once

#include "Point2d.h"
#include "Point3d.h"
#include "../../../Axis.h"

class Endstop;
namespace tools {
  class Tool;
}

const int useDefaultFeedrate = -1;
const float useDefaultMaxTravel = 9999.9f;
const float useDefaultRetractDistance = -1.0f;
float getDefaultFeedrate();

int outputMovementStatus();

// Enqueue movements (do not wait for movement to finish)
int asyncRelativeMove(tools::Tool& tool, float x, float y, float z, float e, float f = useDefaultFeedrate);

// Absolute movement (synchronized)
int moveXY(tools::Tool& tool, float x, float y, float f = useDefaultFeedrate);
int moveZ(tools::Tool& tool, float z, float f = useDefaultFeedrate);

// Relative movement (synchronized)
int relativeMove(tools::Tool& tool, float x, float y, float z, float e, float f = useDefaultFeedrate);

// Move until switch hit
int moveToLimit(AxisEnum axis, int direction, float f = useDefaultFeedrate, float maxTravel = useDefaultMaxTravel);
int moveToEndstop(const Endstop& endstop, float f = useDefaultFeedrate, float maxTravel = useDefaultMaxTravel);

// Raise
// NOTE: We believe that raising to the zMax endstop can
//       introduce a small amount to error into the z-axis
//       so raiseToSoftMax should be used instead
int raiseToSoftMax(tools::Tool& tool);
int raise(tools::Tool& tool);
int raiseToEndstop();

// Set planner position
int setPosition(float x, float y, float z, float e);


int retractFromSwitch(const Endstop& endstop, float retractDistance = useDefaultRetractDistance);


// TODO: use a namespace, e.g. movement::raw movement::internal

const bool skipMovementSafetyCheck = false;

int asyncRawMove(float x, float y, float z, float e, float f, bool confirmMoveIsSafe = true);

// synchronized relative movement (raw means no endstop checks)
int relativeRawMoveXYZ(float x, float y, float z, float speed_in_mm_per_min = useDefaultFeedrate, bool confirmMoveIsSafe = true);
int relativeRawMoveE(float e, float speed_in_mm_per_min = useDefaultFeedrate);

int ensureHomedInXY(tools::Tool& tool);
int centerTool(tools::Tool& tool);

const float NoRetract = -9999.0f;

int confirmAttached(const char* context, tools::Tool& tool);

// step/millimeter conversion
extern float axis_steps_per_unit[4];
float stepsToMillimeters(long step, AxisEnum axis);
long millimetersToSteps(float mm, AxisEnum axis);
