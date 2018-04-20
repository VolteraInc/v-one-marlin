#include "../../Marlin.h"
#include "../../stepper.h"
#include "../../planner.h"
#include "../api/api.h"
#include "../vone/vone.h"
#include "work.h"

void checkForEndstopHits() {
  bool triggered[3];
  long stepsWhenTriggered[3];
  if (readAndResetEndstops(triggered, stepsWhenTriggered)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Limit switch triggered unexpectedly, clean up, then report error");
    SERIAL_EOL;

    // Reset homing and tool preparations
    // Note: We might not need to reset all the axes, but it's more robust to do so.
    setHomedState(X_AXIS, 0);
    setHomedState(Y_AXIS, 0);
    setHomedState(Z_AXIS, 0);
    vone->toolBox.currentTool().resetPreparations();

    // Output an error
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to complete movement, limit switch triggered. Motion stopped at");
    if (triggered[ X_AXIS ]) {
      SERIAL_ERRORPGM(" x:"); SERIAL_ERROR((float)stepsWhenTriggered[X_AXIS] / axis_steps_per_unit[X_AXIS]);
    }
    if (triggered[ Y_AXIS ]) {
      SERIAL_ERRORPGM(" y:"); SERIAL_ERROR((float)stepsWhenTriggered[Y_AXIS] / axis_steps_per_unit[Y_AXIS]);
    }
    if (triggered[ Z_AXIS ]) {
      SERIAL_ERRORPGM(" z:"); SERIAL_ERROR((float)stepsWhenTriggered[Z_AXIS] / axis_steps_per_unit[Z_AXIS]);
    }
    SERIAL_EOL;

    flushAndRequestResend();
  }
}
