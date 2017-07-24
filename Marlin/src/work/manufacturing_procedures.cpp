#include "../../Marlin.h"
#include "../../temperature.h"
#include "../api/api.h"
#include "../api/diagnostics/diagnostics.h"

static auto s_runAt = 0ul;

void manufacturing_init() {
  s_runAt = millis() + 2000ul; // run in 2 sec (give system time to detect tool)
}

static void s_start() {
  overrideLeds(0, 255, 0, 0); // blink green
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Starting manufacturing procedure");
}

static int s_end(Tool tool, int result) {
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

int runBurnInSequence(Tool tool) {
  s_start();
  return s_end(tool, burnInSequence(tool));
}

int runCalibrateSwitchPositions(Tool tool, unsigned cycles) {
  s_start();
  return s_end(tool, calibrateSwitchPositions(tool, cycles));
}

static int s_run() {
  const auto tool = getTool();
  switch (tool) {
    case TOOLS_PROBE: return runCalibrateSwitchPositions(tool);
    case TOOLS_NONE:  return runBurnInSequence(tool);

    default:
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Notice: No manufacturing procedures executed, resume normal operation");
      return 0;
  }
}

void manufacturing_procedures() {
  if (s_runAt == 0 || s_runAt > millis()) {
    return;
  }
  s_runAt = 0;

  // Check z-switch to determine if we should enter manufactuing mode
  // Note: we check then raise then check again just in case
  // a tool is contacting the switch. if it is, then raising is
  // good idea.
  if (READ_PIN(Z_MIN)) {
    raise();
    if (READ_PIN(Z_MIN)) {
      s_run();
    }
  }
}
