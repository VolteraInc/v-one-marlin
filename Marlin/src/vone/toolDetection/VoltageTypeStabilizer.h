#pragma once

#include "VoltageType.h"

namespace toolDetection {

// Provides stabilized classification of p-top voltages
// Note: The p-top pin has a capacitor. This means the voltage
//       falls quickly, but climbs slowly.
//       e.g climbing from 3.5 to 5 take about 30ms.
class VoltageTypeStabilizer {
  public:
    void add(unsigned long time, float voltage);
    VoltageType value() { return m_stable ? m_type : VoltageType::Unknown; }

  private:
    class VoltageLog {
      struct Sample {
        unsigned long time;
        VoltageType type;
        float voltage;
      };

      public:
        unsigned int size() const { return m_size; }
        bool empty() const { return m_size == 0; }
        bool full() const { return m_size == MAX_SAMPLES; }

        const Sample& front() const { return m_samples[m_frontIdx]; }
        const Sample& back() const { return m_samples[m_backIdx]; }

        void push(unsigned long time, VoltageType type, float voltage);
        void pop();

        unsigned long timespan() const;
        unsigned long timeSpanOfCurrentType() const;
        void output() const;

      private:
        static const unsigned int MAX_SAMPLES = 12;
        Sample m_samples[MAX_SAMPLES];
        unsigned int m_size = 0;
        unsigned int m_frontIdx = 0;
        unsigned int m_backIdx = 0;

        static inline unsigned int increment(unsigned int idx) {
          return idx == MAX_SAMPLES - 1 ? 0 : idx + 1;
        }

        static inline unsigned int decrement(unsigned int idx) {
          return idx == 0 ? MAX_SAMPLES - 1 : idx - 1;
        }
    };

    VoltageLog m_voltages;
    bool m_stable = false;
    unsigned long m_unstableTime = 0;
    VoltageType m_type = VoltageType::Unknown;

    void setStable(bool stable);
};

}
