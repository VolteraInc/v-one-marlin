#pragma once

#include "../../../Axis.h"

namespace tools {
  class Tool;
}

bool homedXY();
bool homedZ();

int homeXY(tools::Tool& tool);

int homeZOnly(tools::Tool& tool, float offset = 0.0f);
int homeZandEstablishSoftMax(tools::Tool& tool, float offset = 0.0f);

int getHomedState(AxisEnum axis);
void setHomedState(AxisEnum axis, int value);
void sendHomedStatusUpdate();

int moveToZSwitchXY(tools::Tool& tool);

// Unsafe methods for debugging + gcode commands
// NOTE: raw/unsafe because they these do not raise before homing
int rawHomeX();
int rawHomeY();
