#pragma once

#include "VoltateType.h"

// Provides stabilized classification of p-top voltages
// Note: The p-top pin has a capacitor. This means the voltage
//       falls quickly, but climbs slowly.
//       e.g climbing from 3.5 to 5 take about 30ms.
class VoltageTypeStabilizer {
  public:
    VoltageTypeStabilizer();

    void add(PTopPin::Sample);
    VoltageType value();

  private:
    unsigned int m_count = 0;
    unsigned long startTime;
    auto currentType = VoltageType::Unknown;

    static const unsigned int warningThreshold = 12;
    float voltages[warningThreshold];
};
