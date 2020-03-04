#include "Probe.h"

#include "../../../Marlin.h"
#include "../../api/api.h"
#include "../../api/measurement/measurement.h"
#include "../../api/movement/movement.h"
#include "../pins/PTopPin/PTopPin.h"
#include "../toolDetection/classifyVoltage.h"
#include "../stepper/Stepper.h"
#include "../endstops/ScopedEndstopEnable.h"

int confirmAttachedAndNotTriggered(const char* context, tools::Probe& probe) {
  if (confirmAttached(context, probe)) {
    return -1;
  }
  if (probe.readAnalogTriggered()) {
    logError
      << F("Unable to ") << context
      << F(", probe reported contact before movement started") << endl;
    return -1;
  }
  return 0;
}

tools::Probe::Probe(
  Stepper& stepper,
  PTopPin& pin,
  const Endstop& toolSwitch,
  const ZSwitch& zSwitch
)
  : Tool(stepper)
  , m_pin(pin)
  , m_toolSwitch(toolSwitch)
  , m_zSwitch(zSwitch)
{
}

int tools::Probe::prepareToMoveImpl_Start() {
  m_stepper.endstopMonitor.ignore(m_toolSwitch, false);
  return (
    raiseToEndstop() ||
    confirmAttachedAndNotTriggered("prepare probe", *this)
  );
}

int tools::Probe::prepareToMoveImpl_HomeXY() {
  return ensureHomedInXY(*this);
}

int tools::Probe::prepareToMoveImpl_CalibrateXYZ() {
  auto& endstopMonitor = m_stepper.endstopMonitor;

  if (!m_zSwitch.isSpringStrongerThanProbe()) {
    // Using original z-switch, which has a spring weaker than
    // the probe's. So, we expect the z-switch to trigger,
    // and report an error if the probe triggers first.
    // NOTE:
    //   1) a nice side-effect is that we will detect bad z-switches
    //   2) with this zSwitch the probe's spring will compress
    //      slightly, resulting in variablity the homed Z position
    return (
      homeZOnly(*this) || // home Z so we can enter the xy pos with decent precision
      centerTool(*this) ||
      measureProbeDisplacement(*this, m_probeDisplacement) || // do this now, saves a trip back to the xy-pos after re-homing
      homeZandEstablishSoftMax(*this) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)
      raiseToSoftMax(*this)
    );
  }

  // We are using a z-switch with a spring stronger than the probe
  // so we will descend and trigger the probe before starting the
  // z-homing sequence.

  // Confirm the probe triggers before the z-switch
  {
    log << F("Lowering probe toward z-switch until ") << m_toolSwitch.name << F(" triggers")<< endl;
    ScopedEndstopEnable scopedEnable(endstopMonitor, m_zSwitch);
    if (
      moveToZSwitchXY(*this) ||
      moveToEndstop(m_toolSwitch)
    ) {
      return -1;
    }
  }

  // Ignore the toolswitch during z-homing
  ScopedEndstop_DISABLE sed(endstopMonitor, m_toolSwitch);

  return (
    // home Z so we can enter the xy pos with decent precision
    homeZ(*this, 0.250) ||
    centerTool(*this) ||
    measureProbeDisplacement(*this, m_probeDisplacement) ||

    // Re-home at a consistent xy location
    // (and compensate for probe displacement)
    // NOTE: we've seen a 30um differnce in the
    //       measurement with the old switch
    homeZandEstablishSoftMax(*this, m_probeDisplacement) ||

    raiseToSoftMax(*this)
  );
}

int tools::Probe::resetPreparationsImpl() {
  enableHeightSafety(false);
  setHomedState(Z_AXIS, 0);
  m_stepper.endstopMonitor.ignore(m_toolSwitch);
  return 0;
}

int tools::Probe::enqueueMove(float x, float y, float z, float e, float f) {
  return asyncRawMove(x, y, z, e, f);
}

bool tools::Probe::isTriggered(float voltage) {
  using namespace toolDetection;
  return classifyVoltage(voltage) == VoltageType::ProbeTriggered;
}

bool tools::Probe::readAnalogTriggered(float* o_voltageReading) {
  const auto voltage = m_pin.readValue().voltage;
  if (o_voltageReading) {
    *o_voltageReading = voltage;
  }
  return tools::Probe::isTriggered(voltage);
}

float tools::Probe::displacement() const {
  return m_probeDisplacement;
}

int tools::Probe::probe(
  float& measurement,
  float speed,
  float additionalRetractDistance,
  unsigned maxSamples,
  unsigned maxTouchesPerSample,
  unsigned* o_samplesTaken,
  unsigned* o_touchesUsed
) {
  const auto startTime = millis();

  if (confirmAttachedAndNotTriggered("probe", *this)) {
    return -1;
  }

  float rawMeasurement;
  auto samplesTaken = 0u;
  auto totalTouches = 0u;
  if (
    // Get close to surface using a fast-touch
    // Note: could make this conditional on probe speed,
    // but I'd rather not have the extra code path
    fastTouch(m_toolSwitch) ||

    // Measure
    multiMultiTouch(
      "probe",
      m_toolSwitch,
      rawMeasurement,
      speed,
      maxSamples, maxTouchesPerSample,
      &samplesTaken, &totalTouches
    )
  ) {
    return -1;
  }

  // Return to a safe travel height (unless 'NoRetract' given)
  if (additionalRetractDistance != NoRetract) {
    if (retractFromSwitch(m_toolSwitch, m_probeDisplacement + additionalRetractDistance)) {
      return -1;
    }
  }

  // Move to safe height, if enabled
  if (heightSafetyEnabled()) {
    updateSafeHeight(current_position[Z_AXIS]);
    if (moveToSafeHeight()) {
      return -1;
    }
  }

  // Success
  // TODO: should round to nearest step
  measurement = rawMeasurement + m_probeDisplacement;
  if (o_samplesTaken) { *o_samplesTaken = samplesTaken; }
  if (o_touchesUsed) { *o_touchesUsed = totalTouches; }
  if (logging_enabled) {
    const auto duration = millis() - startTime;
    log
      << F("probe height: ") << rawMeasurement
      << F(", displacement: ") << m_probeDisplacement
      << F(", measurement: ") << measurement
      << F(", samplesTaken: ") << samplesTaken
      << F(", totalTouches: ") << totalTouches
      << F(", duration: ") << duration
      << endl;
  }
  return 0;
}


// ----------------------------------------------
// Height safety

void tools::Probe::enableHeightSafety(bool enable) {
  m_heightSafetyEnabled = enable;

  // Reset safe height on disable
  if (!m_heightSafetyEnabled) {
    m_numHeightSamples = 0;
    m_maxSampledHeight = -INFINITY;
    m_safeHeight = -INFINITY;
  }
}

void tools::Probe::updateSafeHeight(float height) {
  // If we have a value safe height, update it
  if (!isinf(m_safeHeight)) {
    m_safeHeight = max(m_safeHeight, height);
    return;
  }

  // Use the max of 3 height samples to establish a safe height
  ++m_numHeightSamples;
  m_maxSampledHeight = max(m_maxSampledHeight, height);
  if (m_numHeightSamples == 3) {
    m_safeHeight = m_maxSampledHeight;
  }
}

float tools::Probe::safeHeight() const {
  return isinf(m_safeHeight) ? max_pos[Z_AXIS] - 1.0 : m_safeHeight;
}

int tools::Probe::moveToSafeHeight() {
  // If height safety is disabled, do nothing
  if (!heightSafetyEnabled()) {
    return 0;
  }

  // move to safe height
  return moveZ(*this, safeHeight());
}

// ----------------------------------------------
// Status

void tools::Probe::outputStatus() const {
  log << F("Probe") << endl;
  log << F("  displacement: ") << displacement() << endl;

  if (heightSafetyEnabled()) {
    log << F("  height safety: ON, safe height is ") << safeHeight() << endl;
  } else {
    log << F("  height safety: inactive") << endl;
  }
}
