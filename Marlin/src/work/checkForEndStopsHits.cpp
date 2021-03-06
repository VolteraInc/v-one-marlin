#include "../../Marlin.h"
#include "../vone/VOne.h"
#include "work.h"

static void s_reportHit(const Endstop& endstop, float position) {
  const auto axis = endstop.axis;
  logError
    << F("Unable to complete movement, ")
    << endstop.name
    << F(" triggered at ")
    << axis_codes[axis]
    << F("=")
    << position
    << endl;
}

void checkForEndstopHits() {
  auto& stepper = vone->stepper;
  const auto& endstops = vone->endstops;
  if (!stepper.endstopMonitor.hasUnreportedHits()) {
    return;
  }

  // Output a context message
  log
    << F("Limit switch triggered unexpectedly, cleaning up, before reporting error")
    << endl;

  // Stop/reset everything
  // Note: We might not need to reset all the axes, but it's more robust to do so.
  stepper.stop(F("limit switch triggered unexpectedly"));
  setHomedState(X_AXIS, 0);
  setHomedState(Y_AXIS, 0);
  setHomedState(Z_AXIS, 0);
  vone->toolBox.currentTool().resetPreparations();

  // Report Errors
  stepper.endstopMonitor.reportHits(s_reportHit);

  // Output current switch status (to aid with debugging)
  log
    << F("Note: the following switch statuses are read direct from ")
    << F("the switches, if they do not match the error above then ")
    << F("a switch may be triggering intermittently")
    << endl;
  endstops.outputStatus();

  // Flush inbound commands
  // Note: will conditionally request a resend
  flushSerialCommands();
}
