#pragma once

#include "VoltageType.h"

class PTopPin {
  public:
    class Sample;
};

namespace toolDetection {

// Provides stabilized classification of p-top voltages
// Note: The p-top pin has a capacitor. This means the voltage
//       falls quickly, but climbs slowly.
//       e.g climbing from 3.5 to 5 take about 30ms.
class VoltageTypeStabilizer {
  public:
    VoltageTypeStabilizer();

    void add(PTopPin::Sample& sample);
    VoltageType value() { return m_stableType; }

  private:
    unsigned int m_count = 0;
    unsigned long m_previousSampleTime;
    VoltageType m_type = VoltageType::Unknown;
    VoltageType m_stableType = VoltageType::Unknown;

    static const unsigned int warningThreshold = 12;
    float voltages[warningThreshold];
};

}
