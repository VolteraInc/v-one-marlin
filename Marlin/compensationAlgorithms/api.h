void applyScalingCompensation(float& x, float& y, float xScale, float yScale);

void applySkewCompensation(float& x, float& y, float cosTheta, float tanTheta);

long applyBacklashCompensation(int direction, long steps, long offset);
