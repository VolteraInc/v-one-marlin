#pragma once

struct Point2D {
  float x = 0.0f;
  float y = 0.0f;
};

int prepareProbe(Tool tool, Point2D& reference);
int prepareDispenser(Tool tool, const Point2D& reference);
