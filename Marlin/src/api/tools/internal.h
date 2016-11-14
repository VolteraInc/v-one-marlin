#pragma once

struct Point2D {
  float x = 0.0f;
  float y = 0.0f;
};

int prepareProbe(Tool tool);
int prepareDispenser(Tool tool);
bool probeMounted();