#include "../../..//Marlin.h"
#include "../../api/api.h"
#include "../../api/diagnostics/diagnostics.h"

#include "../../../planner.h"

#include "../../vone/tools/NullTool.h"
#include "../../vone/tools/Probe.h"

static void s_start() {
  overrideLeds(0, 255, 0, 0); // blink green
  log << F("Starting manufacturing procedure") << endl;
}

static int s_end(tools::Tool& tool, int result) {
  if (result == 0) {
    const float bedCenterX = min_pos[X_AXIS] + (max_pos[X_AXIS] - min_pos[X_AXIS]) / 2;

    // Success, pulse leds and go to finished position
    // (trying to make it obvious that we are done)
    log << F("Manufacturing procedure completed successfully") << endl;
    raise();
    moveXY(tool, bedCenterX, max_pos[Y_AXIS]);

    overrideLeds(255, 255, 255, 3); // pulse white (which also helps confirm LEDs have all 3 colors)
  } else {
    logError << F("Manufacturing procedure failed") << endl;
    overrideLeds(255, 80, 0, 0); // blink yellow
  }
  return result;
}

int runBurnInSequence(tools::NullTool& noTool) {
  s_start();
  return s_end(
    noTool,
    burnInSequence(noTool)
  );
}

int runCalibrateSwitches(tools::Probe& probe, unsigned cycles) {
  s_start();
  return s_end(
    probe,
    calibrateSwitchPositions(probe, cycles) ||
    checkBackSwitchSeparation(probe) ||
    checkExtents(probe)
  );
}
