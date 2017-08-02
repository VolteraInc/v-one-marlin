#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../../../temperature.h"
#include "../api.h"
#include "../measurement/measurement.h"
#include "../switches/PTopScopedUsageLock.h"
#include "internal.h"

static float s_probeDisplacement = 0.0f;

int probe::partiallyPrepareProbe(const char* context, Tool tool) {
  enable_p_top(true);
  return (
    raise() ||
    confirmMountedAndNotTriggered(context, tool, TOOLS_PROBE) ||
    ensureHomedInXY()
  );
}

int prepareProbe(Tool tool) {
  const char* context = "prepare probe";
  return (
    probe::partiallyPrepareProbe(context, tool) ||
    homeZ(tool) || // home Z so we can enter the xy pos with decent precision
    centerTool(tool) ||
    measureProbeDisplacement(tool, s_probeDisplacement) || // do this now, saves a trip back to the xy-pos after re-homing
    homeZ(tool) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)
    raise()
  );
}

bool probe::isTriggered(float voltage) {
  return classifyVoltage(TOOLS_PROBE, voltage) == TOOL_STATE_TRIGGERED;
}

float probe::getProbeDisplacement() {
  return s_probeDisplacement;
}

int probe::probe(
  Tool tool,
  float& measurement,
  float speed,
  float additionalRetractDistance,
  unsigned maxSamples,
  unsigned maxTouchesPerSample,
  unsigned* o_samplesTaken,
  unsigned* o_touchesUsed
) {

  if (confirmMountedAndNotTriggered("probe", tool, TOOLS_PROBE)) {
    return -1;
  }

  // Disable analog reads to prevent false tool change detection
  // (due to the unusualy amount to time the probe will be intermittently triggered)
  PTopScopedUsageLock scopedUse;

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
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("probe height: ", rawMeasurement);
    SERIAL_ECHOPAIR(", displacement: ", s_probeDisplacement);
    SERIAL_ECHOPAIR(", measurement: ", measurement);
    SERIAL_ECHOPAIR(", samplesTaken: ", samplesTaken);
    SERIAL_ECHOPAIR(", totalTouches: ", totalTouches);
    SERIAL_EOL;
  }
  return 0;
}
