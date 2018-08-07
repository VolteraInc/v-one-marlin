#include "../../Marlin.h"
#include "../api/api.h"
#include "../api/diagnostics/diagnostics.h"
#include "../vone/VOne.h"

static auto s_runAt = 0ul;

void manufacturing_init() {
  s_runAt = millis() + 2000ul; // run in 2 sec (give system time to detect tool)
}

static int s_run() {
  if (vone->toolBox.probe.attached()) {
    return runCalibrateSwitches(vone->toolBox.probe);
  } else if (vone->toolBox.nullTool.attached()) {
    return runBurnInSequence(vone->toolBox.nullTool);
  } else {
    logNotice << F("No manufacturing procedures executed, resume normal operation") << endl;
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
    auto& currentTool = vone->toolBox.currentTool();

    if (
      currentTool.prepareToMove(tools::PrepareToMove::Options::startOnly) ||
      raise()
    ) {
      logWarning << F("Unable to release z-switch on boot, ignoring") << endl;
      return;
    }

    if (READ_PIN(Z_MIN)) {
      s_run();
    }
  }
}
