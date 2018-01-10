#include "Probe.h"

#include "../../../Marlin.h"
#include "../../../stepper.h" // enable_p_top TODO: go through Stepper
#include "../../api/api.h"
#include "../../api/measurement/measurement.h"
#include "../../api/movement/movement.h"
#include "../pins/PTopPin/PTopPin.h"
#include "../toolDetection/classifyVoltage.h"

int confirmAttachedAndNotTriggered(const char* context, tools::Probe& probe) {
  if (confirmAttached(context, probe)) {
    return -1;
  }
  if (probe.readAnalogTriggered()) {
    SERIAL_ERROR_START;
    SERIAL_PAIR("Unable to ", context); SERIAL_ERRORPGM(", probe reported contact before movement started");
    SERIAL_EOL;
    return -1;
  }
  return 0;
}

tools::Probe::Probe(Stepper& stepper, PTopPin& pin)
  : Tool(stepper)
  , m_pin(pin)
{
}

int tools::Probe::partiallyPrepare(const char* context) {
  enable_p_top(true);
  return (
    raise() ||
    confirmAttachedAndNotTriggered(context, *this) ||
    ensureHomedInXY(*this)
  );
}

int tools::Probe::prepareToMoveImpl() {
  const char* context = "prepare probe";
  return (
    tools::Probe::partiallyPrepare(context) ||
    homeZ(*this) || // home Z so we can enter the xy pos with decent precision
    centerTool(*this) ||
    measureProbeDisplacement(*this, m_probeDisplacement) || // do this now, saves a trip back to the xy-pos after re-homing
    homeZ(*this) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)
    raise()
  );
}

int tools::Probe::resetPreparationsImpl() {
  setHomedState(Z_AXIS, 0);
  enable_p_top(false);
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
    fastTouch() ||

    // Measure
    multiMultiTouch(
      "probe",
      rawMeasurement,
      speed,
      maxSamples, maxTouchesPerSample,
      &samplesTaken, &totalTouches
    ) ||

    // Return to a safe travel height
    // TODO: should return to height we were at when we started OR ???
    retractToolConditionally(m_probeDisplacement, additionalRetractDistance)
  ) {
    return -1;
  }

  // Success
  // TODO: should round to nearest step
  measurement = rawMeasurement + m_probeDisplacement;
  if (o_samplesTaken) { *o_samplesTaken = samplesTaken; }
  if (o_touchesUsed) { *o_touchesUsed = totalTouches; }
  if (logging_enabled) {
    const auto duration = millis() - startTime;
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("probe height: ", rawMeasurement);
    SERIAL_ECHOPAIR(", displacement: ", m_probeDisplacement);
    SERIAL_ECHOPAIR(", measurement: ", measurement);
    SERIAL_ECHOPAIR(", samplesTaken: ", samplesTaken);
    SERIAL_ECHOPAIR(", totalTouches: ", totalTouches);
    SERIAL_ECHOPAIR(", duration: ", duration);
    SERIAL_EOL;
  }
  return 0;
}
