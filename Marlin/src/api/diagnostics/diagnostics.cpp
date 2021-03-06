#include "../../..//Marlin.h"
#include "../../api/api.h"
#include "../../api/diagnostics/diagnostics.h"

#include "../../vone/tools/NullTool.h"
#include "../../vone/tools/Probe.h"

static int s_beginDiagnotic(tools::Tool& tool) {
  overrideLeds(0, 255, 0, 3); // pulse green
  log << F("Starting diagnostic procedure") << endl;

  if (
    tool.prepareToMove(tools::PrepareToMove::Option::startOnly) ||

    // Ensure the tool is not pressing the z-switch
    raise(tool) ||

    // Give the user time to release the z-switch
    // Note: If the z-switch is still triggered after
    //       the delay it's probably stuck
    relativeMove(tool, 0, 0, -0.5, 0, 10)
  ) {
    return -1;
  }

  if (READ_PIN(Z_MIN)) {
    logError << F("Unable to complete diagnostic procedure, Z-switch reported as 'triggered' after raising carriage") << endl;
    return -1;
  }

  overrideLeds(0, 255, 0, 0); // blink green
  return 0;
}

static int s_handleResult(tools::Tool& tool, int result) {
  if (result == 0) {
    const float bedCenterX = min_pos[X_AXIS] + (max_pos[X_AXIS] - min_pos[X_AXIS]) / 2;

    // Success, pulse leds and go to finished position
    // (trying to make it obvious that we are done)
    log << F("Diagnostic procedure completed successfully") << endl;
    raise(tool);
    moveXY(tool, bedCenterX, max_pos[Y_AXIS]);

    overrideLeds(255, 255, 255, 3); // pulse white (which also helps confirm LEDs have all 3 colors)
  } else {
    logError << F("Diagnostic procedure failed") << endl;
    overrideLeds(255, 80, 0, 0); // blink yellow
  }
  return result;
}

int runBurnInSequence(tools::NullTool& noTool) {
  int result = (
    s_beginDiagnotic(noTool) ||
    burnInSequence(noTool)
  );
  return s_handleResult(noTool, result);
}

int runCalibrateSwitches(tools::Probe& probe, unsigned cycles) {
  int result = (
    s_beginDiagnotic(probe) ||
    calibrateSwitchPositions(probe, cycles) ||
    checkBackSwitchSeparation(probe) ||
    checkExtents(probe)
  );
  return s_handleResult(probe, result);
}
