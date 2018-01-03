#include "../../..//Marlin.h"
#include "../../api/api.h"
#include "../../api/diagnostics/diagnostics.h"

#include "../../vone/tools/NullTool.h"
#include "../../vone/tools/Probe.h"

static void s_start() {
  overrideLeds(0, 255, 0, 0); // blink green
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Starting manufacturing procedure");
}

static int s_end(tools::Tool& tool, int result) {
  if (result == 0) {
    const float bedCenterX = min_pos[X_AXIS] + (max_pos[X_AXIS] - min_pos[X_AXIS]) / 2;

    // Success, pulse leds and go to finished position
    // (trying to make it obvious that we are done)
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Manufacturing procedure completed successfully");
    raise();
    moveXY(tool, bedCenterX, max_pos[Y_AXIS]);

    overrideLeds(255, 255, 255, 3); // pulse white (which also helps confirm LEDs have all 3 colors)
  } else {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Manufacturing procedure failed");
    overrideLeds(255, 80, 0, 0); // blink yellow
  }
  return result;
}

int runBurnInSequence(tools::NullTool& noTool) {
  s_start();
  return s_end(noTool, burnInSequence(noTool));
}

int runCalibrateSwitchPositions(tools::Probe& probe, unsigned cycles) {
  s_start();
  return s_end(probe, calibrateSwitchPositions(probe, cycles));
}
