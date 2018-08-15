void applyScalingCompensation(float& x, float& y, float xScale, float yScale);
float reverseScalingCompensation(float p, float axisScalingFactor);

void applySkewCompensation(float& x, float& y, float cosTheta, float tanTheta);
float reverseSkewCompensationInX(float x, float cosTheta);
float reverseSkewCompensationInY(float x, float y, float tanTheta);

long applyBacklashCompensation(const char* label, int direction, long steps, long offset);
