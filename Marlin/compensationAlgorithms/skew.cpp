void applySkewCompensation(float& x, float& y, float cosTheta, float tanTheta) {
  y = (y - x * tanTheta);
  x = x / cosTheta;
}
