#include "../../Marlin.h"
#include "../api/api.h"
#include "../api/diagnostics/diagnostics.h"

static auto s_runAt = 0ul;

void manufacturing_init() {
  s_runAt = millis() + 2000ul; // run in 2 sec (give system time to detect tool)
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
