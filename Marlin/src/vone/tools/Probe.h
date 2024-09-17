#pragma once
#include <Arduino.h>
#include "Tool.h"

class Endstop;
class ZSwitch;
class PTopPin;

namespace tools {

class Probe : public Tool {
  public:
    Probe(
      Stepper& stepper,
      PTopPin& pin,
      const Endstop& toolSwitch,
      const ZSwitch& zSwitch
    );

    // Retract constants
    // Note: Retract enough to ensure we don't crash when
    // moving across the board. We've used 0.7mm for
    // years without issue.
    static constexpr float DefaultRetract = 0.7f;

    static constexpr float DefaultSpeed = 200;
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
      unsigned* o_touchesUsed = nullptr,
      bool oldProbe = false
    );

    float displacement() const;
    bool isTriggered(float voltage);
    bool readAnalogTriggered(float* o_voltageReading = nullptr);

    bool heightSafetyEnabled() const { return m_heightSafetyEnabled; }
    void enableHeightSafety(bool enable = true);
    float safeHeight() const;
    int moveToSafeHeight();

    void outputStatus() const;

  private:
    float m_probeDisplacement = 0.0f;
    PTopPin& m_pin;
    const Endstop& m_toolSwitch;
    const ZSwitch& m_zSwitch;
    bool m_heightSafetyEnabled = false;
    unsigned int m_numHeightSamples = 0u;
    float m_maxSampledHeight = -INFINITY;
    float m_safeHeight = -INFINITY;

    virtual const char* type() const override { return "Probe"; }
    virtual unsigned int version() const override { return 1; }

    virtual int prepareToMoveImpl_Start() override;
    virtual int prepareToMoveImpl_HomeXY() override;
    virtual int prepareToMoveImpl_CalibrateXYZ() override;
    virtual int resetPreparationsImpl() override;
    virtual int enqueueMove(float x, float y, float z, float e, float f) override;

    void updateSafeHeight(float height);
};

}
