#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../planner.h"
#include "../api/api.h"
#include "../vone/vone.h"
#include "work.h"

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
  logError
    << F("Unable to complete movement, limit switch triggered in")
    << (triggered[ X_AXIS ] ? F(" X") : F(""))
    << (triggered[ Y_AXIS ] ? F(" Y") : F(""))
    << (triggered[ Z_AXIS ] ? F(" Z") : F(""))
    << ", position is "
    << (float)stepsWhenTriggered[X_AXIS] / axis_steps_per_unit[X_AXIS]
    << (float)stepsWhenTriggered[Y_AXIS] / axis_steps_per_unit[Y_AXIS]
    << (float)stepsWhenTriggered[Z_AXIS] / axis_steps_per_unit[Z_AXIS]
    << endl;
}
