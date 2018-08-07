#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../planner.h"
#include "../api/api.h"
#include "../vone/VOne.h"
#include "work.h"

static void s_outputError(int axis, long stepsWhenTriggered[3]) {
  logError
    << F("Unable to complete movement, limit switch triggered in ")
    << axis_codes[axis]
    << F(" at position ")
    << (float)stepsWhenTriggered[axis] / axis_steps_per_unit[axis]
    << endl;
}

void checkForEndstopHits() {
  bool triggered[3];
  long stepsWhenTriggered[3];
  if (!readAndResetEndstops(triggered, stepsWhenTriggered)) {
    return;
  }

  // Output a context message
  log
    << F("Limit switch triggered unexpectedly, cleaning up, before reporting error")
    << endl;

  // Output switch status
  log
    << F("Note: the following switch statuses are read direct from ")
    << F("the switches, if they do not match the error below then ")
    << F("the switch may be triggering intermittently")
    << endl;
  vone->pins.outputEndStopStatus();

  // Stop/reset everything
  // Note: We might not need to reset all the axes, but it's more robust to do so.
  vone->stepper.stop();
  setHomedState(X_AXIS, 0);
  setHomedState(Y_AXIS, 0);
  setHomedState(Z_AXIS, 0);
  vone->toolBox.currentTool().resetPreparations();
  flushSerialCommands();

  // Output Error
  // NOTE: should generally see just one of these
  if (triggered[ X_AXIS ]) {
    s_outputError(X_AXIS, stepsWhenTriggered);
  }
  if (triggered[ Y_AXIS ]) {
    s_outputError(Y_AXIS, stepsWhenTriggered);
  }
  if (triggered[ Z_AXIS ]) {
    s_outputError(Z_AXIS, stepsWhenTriggered);
  } 
}