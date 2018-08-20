#include "../../MarlinConfig.h"

void applySkewCompensation(float& x, float& y, float cosTheta, float tanTheta) {
  y = (y - x * tanTheta);
  x = x / cosTheta;
}

float reverseSkewCompensationInX(float x, float cosTheta) {
  return x * cosTheta;
}

float reverseSkewCompensationInY(float x, float y, float tanTheta) {
  return y + x * sin(atan(tanTheta));
}