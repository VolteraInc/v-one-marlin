inline unsigned long seconds(unsigned long s) {
  return s * 1000ul;
}

inline unsigned long minutes(unsigned long m) {
  return m * seconds(60);
}