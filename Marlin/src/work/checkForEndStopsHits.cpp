#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../planner.h"
#include "../api/api.h"
#include "../vone/VOne.h"
#include "work.h"

static void reportEndstopHit(const Endstop& endstop, long triggeringStep) {
  const auto axis = endstop.axis;
  const auto position = (float)triggeringStep / axis_steps_per_unit[axis];
  logError
    << F("Unable to complete movement, ")
    << endstop.name
    << F(" triggered at position ")
    << position
    << endl;
}

void checkForEndstopHits() {
  auto& stepper = vone->stepper;
  const auto& endstops = vone->endstops;
  if (!stepper.hasUnreportedEndstopHits()) {
    return;
  }

  // Output a context message
  log
    << F("Limit switch triggered unexpectedly, cleaning up, before reporting error")
    << endl;

  // Stop/reset everything
  // Note: We might not need to reset all the axes, but it's more robust to do so.
  stepper.stop();
  setHomedState(X_AXIS, 0);
  setHomedState(Y_AXIS, 0);
  setHomedState(Z_AXIS, 0);
  vone->toolBox.currentTool().resetPreparations();
  flushSerialCommands();

  // Report Errors
  stepper.reportEndstopHits(reportEndstopHit);

  // Output current switch status (to aid with debugging)
  log
    << F("Note: the following switch statuses are read direct from ")
    << F("the switches, if they do not match the error above then ")
    << F("a switch may be triggering intermittently")
    << endl;
  endstops.outputStatus();
}
