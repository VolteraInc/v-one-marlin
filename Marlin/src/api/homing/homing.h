#pragma once

#include "../../../Axis.h"

namespace tools {
  class Tool;
}

bool homedXY();
bool homedZ();

int homeXY(tools::Tool& tool);
int homeZ(tools::Tool& tool, float offset = 0.0f);

int getHomedState(AxisEnum axis);
void setHomedState(AxisEnum axis, int value);
void sendHomedStatusUpdate();

int rawHome(tools::Tool& tool, bool homeX = true, bool homeY = true, bool homeZ = true);

int moveToZSwitchXY(tools::Tool& tool);
