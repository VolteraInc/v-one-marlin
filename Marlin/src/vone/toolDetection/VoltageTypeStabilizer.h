#pragma once

#include "VoltageType.h"

namespace toolDetection {

// Provides stabilized classification of p-top voltages
// Note: The p-top pin has a capacitor. This means the voltage
//       falls quickly, but climbs slowly.
//       e.g climbing from 3.5 to 5 take about 30ms.
class VoltageTypeStabilizer {
  public:
    void add(unsigned int time, float voltage);
    VoltageType value() { return m_stable ? m_type : VoltageType::Unknown; }

  private:
    class VoltageLog {
      struct Sample {
        unsigned int time;
        VoltageType type;
        float voltage;
      };

      public:
        bool empty() const { return m_writeIdx == m_readIdx; }
        const Sample& front() const { return m_samples[m_readIdx]; }
        const Sample& back() const { return m_samples[m_writeIdx]; }
        void push(unsigned int time, VoltageType type, float voltage);
        void pop();

        unsigned int timespan() const;
        unsigned int timeSpanOfCurrentType() const;
        void output() const;

      private:
        static const unsigned int MAX_SAMPLES = 12;
        Sample m_samples[MAX_SAMPLES];
        unsigned int m_readIdx = 0;
        unsigned int m_writeIdx = 0;

        static inline void increment(unsigned int& idx) {
          ++idx;
          if (idx == MAX_SAMPLES) {
            idx = 0;
          }
        }

        static inline void decrement(unsigned int& idx) {
          if (idx == 0) {
            idx = MAX_SAMPLES - 1;
          } else {
            --idx;
          }
        }
    };

    VoltageLog m_voltages;
    bool m_stable = false;
    unsigned int m_unstableTime = 0;
    VoltageType m_type = VoltageType::Unknown;

    void setStable(bool stable);
};

}
