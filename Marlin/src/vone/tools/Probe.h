#pragma once
#include "Tool.h"

class PTopPin;

namespace tools {

class Probe : public Tool {
    public:
        Probe(PTopPin& pin);
        virtual const char* typeName() override { return "Probe"; } 
        virtual int prepare() override;
        virtual int unprepare() override;

        // Retract constants
        static constexpr float NoRetract = -9999.0f;
        static constexpr float DefaultRetract = 0.2f;

        static constexpr float DefaultSpeed = 30;
        static constexpr float DefaultMaxSamples = 1u; // 30u;
        static constexpr float DefaultMaxTouchesPerSample = 1u; // 10u;
        // TODO: restore multi-touch for beta ^
        
        int probe(
            float& measurement,
            float speed = DefaultSpeed,
            float additionalRetractDistance = DefaultRetract,
            unsigned maxSamples = DefaultMaxSamples,
            unsigned maxTouchesPerSample = DefaultMaxTouchesPerSample,
            unsigned* o_samplesTaken = nullptr,
            unsigned* o_touchesUsed = nullptr
        );

        int partiallyPrepare(const char* context);
        float getProbeDisplacement();
        bool isTriggered(float voltage);
        bool readAnalogTriggered(float* o_voltageReading = nullptr);
          
    private:
        float m_probeDisplacement = 0.0f;
        PTopPin& m_pin;
};

}
