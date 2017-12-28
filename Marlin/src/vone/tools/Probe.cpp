#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../../api/api.h"
#include "../../api/measurement/measurement.h"

Probe::Probe(PTopPin& pin) 
  : m_pin(pin) 
{
}

int Probe::partiallyPrepare(const char* context) {
  enable_p_top(true);
  return (
    raise() ||
    confirmMountedAndNotTriggered(context) ||
    ensureHomedInXY()
  );
}

int Probe::prepare() {
  const char* context = "prepare probe";
  return (
    Probe::partiallyPrepare(context) ||
    homeZ(tool) || // home Z so we can enter the xy pos with decent precision
    centerTool() ||
    measureProbeDisplacement(s_probeDisplacement) || // do this now, saves a trip back to the xy-pos after re-homing
    homeZ(tool) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)
    raise()
  );
}

int Probe::unprepare() {
  setHomedState(Z_AXIS, 0);
  enable_p_top(false);
  return 0;
}

bool Probe::isTriggered(float voltage) {
  return classifyVoltage(voltage) == TOOL_STATE_TRIGGERED;
}

bool Probe::readAnalogTriggered(float* o_voltageReading) {
  const auto voltage = vone->pins.ptop.readValue().voltage;
  if (o_voltageReading) {
    *o_voltageReading = voltage;
  }
  return Probe::isTriggered(voltage);
}

float Probe::getProbeDisplacement() {
  return s_probeDisplacement;
}

int Probe::probe(
  float& measurement,
  float speed,
  float additionalRetractDistance,
  unsigned maxSamples,
  unsigned maxTouchesPerSample,
  unsigned* o_samplesTaken,
  unsigned* o_touchesUsed
) {
  const auto startTime = millis();

  if (confirmMountedAndNotTriggered("probe")) {
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
    retractToolConditionally(s_probeDisplacement, additionalRetractDistance)
  ) {
    return -1;
  }

  // Success
  // TODO: should round to nearest step
  measurement = rawMeasurement + s_probeDisplacement;
  if (o_samplesTaken) { *o_samplesTaken = samplesTaken; }
  if (o_touchesUsed) { *o_touchesUsed = totalTouches; }
  if (logging_enabled) {
    const auto duration = millis() - startTime;
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("probe height: ", rawMeasurement);
    SERIAL_ECHOPAIR(", displacement: ", s_probeDisplacement);
    SERIAL_ECHOPAIR(", measurement: ", measurement);
    SERIAL_ECHOPAIR(", samplesTaken: ", samplesTaken);
    SERIAL_ECHOPAIR(", totalTouches: ", totalTouches);
    SERIAL_ECHOPAIR(", duration: ", duration);
    SERIAL_EOL;
  }
  return 0;
}
