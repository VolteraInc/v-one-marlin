void applyScalingCompensation(float& x, float& y, float xScale, float yScale);

void applySkewCompensation(float& x, float& y, float cosTheta, float tanTheta);

float applyBacklashCompensation(int newDirection, float newCoordinate, float offset);
