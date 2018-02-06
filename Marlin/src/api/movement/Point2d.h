#pragma once
#include <math.h>

class Point2d {
  public:
    float x;
    float y;

  Point2d offset(float x, float y) const {
    return {
      this->x + x,
      this->y + y
    };
  }

  Point2d rotate(
    float theta,
    const Point2d origin
  ) const {
    const auto sinTheta = sin(theta);
    const auto cosTheta = cos(theta);
    const auto x0 = x - origin.x;
    const auto y0 = y - origin.y;
    return {
      origin.x + (x0 * cosTheta) - (y0 * sinTheta),
      origin.y + (x0 * sinTheta) + (y0 * cosTheta)
    };
  }
};