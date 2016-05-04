#pragma once

struct Point2D {
  float x = 0.0f;
  float y = 0.0f;
};

int prepareProbe(Point2D& reference);
int prepareDispenser(const Point2D& reference);