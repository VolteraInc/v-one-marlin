#pragma once
#include "Tool.h"

class PTopPin;

namespace tools {

class Probe : public Tool {
  public:
    Probe(Stepper& stepper, PTopPin& pin);

    // Retract constants
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

    float displacement() const;
    bool isTriggered(float voltage);
    bool readAnalogTriggered(float* o_voltageReading = nullptr);

  private:
    float m_probeDisplacement = 0.0f;
    PTopPin& m_pin;

    virtual const char* name() const override { return "Probe"; }
    virtual int prepareToMoveImpl() override;
    virtual int resetPreparationsImpl() override;
    virtual int enqueueMove(float x, float y, float z, float e, float f) override;
};

}
