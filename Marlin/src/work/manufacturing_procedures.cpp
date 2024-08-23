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

bool s_isZSwitchPressed(){
  auto& endstopMonitor = vone->stepper.endstopMonitor;
  const auto& zSwitch = vone->endstops.zSwitch;

  #ifdef XYZ_STRAIN
    return endstopMonitor.isTriggered(zSwitch);
  #else
    return READ_PIN(Z_MIN);
  #endif
}

void manufacturing_procedures() {
  if (s_runAt == 0 || s_runAt > millis()) {
    return;
  }
  s_runAt = 0;

  // Check z-switch to determine if we should run a manufactuing procedure
  if (s_isZSwitchPressed()) {
    log << F("Z-switch is triggered, raising");
    auto& currentTool = vone->toolBox.currentTool();
    if (
      currentTool.prepareToMove(tools::PrepareToMove::Option::startOnly) ||
      raiseToEndstop()
    ) {
      logWarning << F("Unable to release z-switch on boot, ignoring") << endl;
      return;
    }

    // Note: the tool may have been pressing the switch,
    //       so we re-check it here (after raising)
    if (!s_isZSwitchPressed()) {
      log << F("Released z-switch");
      return;
    }

    // Run manufactuing procedure
    s_run();
  }
}