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
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Limit switch triggered unexpectedly, cleaning up, before reporting error");
  SERIAL_EOL;

  // Output switch status
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Note: the following switch statuses are read direct from ");
  SERIAL_ECHOPGM("the switches, if they do not match the error below then ");
  SERIAL_ECHOPGM("the switch may be triggering intermittently");
  SERIAL_EOL;
  vone->pins.outputEndStopStatus();

  // Stop/reset everything
  // Note: We might not need to reset all the axes, but it's more robust to do so.
  setHomedState(X_AXIS, 0);
  setHomedState(Y_AXIS, 0);
  setHomedState(Z_AXIS, 0);
  vone->toolBox.currentTool().resetPreparations();

  // Output Error
  SERIAL_ERROR_START;
  SERIAL_ERRORPGM("Unable to complete movement, limit switch triggered. Motion stopped at");
  if (triggered[ X_AXIS ]) {
    SERIAL_PAIR(" x:", (float)stepsWhenTriggered[X_AXIS] / axis_steps_per_unit[X_AXIS]);
  }
  if (triggered[ Y_AXIS ]) {
    SERIAL_PAIR(" y:", (float)stepsWhenTriggered[Y_AXIS] / axis_steps_per_unit[Y_AXIS]);
  }
  if (triggered[ Z_AXIS ]) {
    SERIAL_PAIR(" z:", (float)stepsWhenTriggered[Z_AXIS] / axis_steps_per_unit[Z_AXIS]);
  }
  SERIAL_EOL;
}
