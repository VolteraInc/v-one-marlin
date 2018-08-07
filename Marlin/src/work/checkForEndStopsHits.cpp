#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../planner.h"
#include "../api/api.h"
#include "../vone/VOne.h"
#include "work.h"

static void s_outputError(const Endstop& endstop, long stepsWhenTriggered[3]) {
  logError
    << F("Unable to complete movement, ")
    << endstop.name
    << F(" triggered at position ")
    << (float)stepsWhenTriggered[endstop.axis] / axis_steps_per_unit[endstop.axis]
    << endl;
}

void checkForEndstopHits() {
  auto& stepper = vone->stepper;
  bool triggered[3];
  long stepsWhenTriggered[3];
  stepper.stopIfEndstopTriggered();

  if (!stepper.copyEndstopStatus(triggered, stepsWhenTriggered)) {
    return;
  }

  // Output a context message
  log
    << F("Limit switch triggered unexpectedly, cleaning up, before reporting error")
    << endl;

  // Stop/reset everything
  // Note: We might not need to reset all the axes, but it's more robust to do so.
  vone->stepper.stop();
  setHomedState(X_AXIS, 0);
  setHomedState(Y_AXIS, 0);
  setHomedState(Z_AXIS, 0);
  vone->toolBox.currentTool().resetPreparations();
  flushSerialCommands();

  // Output switch status
  log
    << F("Note: the following switch statuses are read direct from ")
    << F("the switches, if they do not match the error below then ")
    << F("the switch may be triggering intermittently")
    << endl;
  vone->endstops.outputStatus();

  // Output Error
  // NOTE: should generally see just one of these
  if (triggered.xMin) {
    s_outputError(endstops.xMin, stepsWhenTriggered);
  }
  ...//TODO: one for each endstop
}