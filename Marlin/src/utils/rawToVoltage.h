#pragma once

inline float rawToVoltage(long value) {
  return 5.0f * value / 1024.0f;
}
