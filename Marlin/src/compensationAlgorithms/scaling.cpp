void applyScalingCompensation(float& x, float& y, float xScale, float yScale) {
  x = x / xScale;
  y = y / yScale;
}

float reverseScalingCompensation(float p, float axisScalingFactor) {
  return p * axisScalingFactor;
}