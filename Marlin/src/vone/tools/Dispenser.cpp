#include "../../../Marlin.h"
#include "../../../planner.h"
#include "../../api/api.h"

namespace tools {

int meshGears() {
  // Move E backward, then move forward.
  return (
    relativeRawMoveE(-0.02) ||
    relativeRawMoveE(0.02)
  );
}

int Dispenser::prepare() {
  const char* context = "prepare dispenser";
  return (
    raise() ||
    confirmMountedAndNotTriggered(context, ToolType::Dispenser) ||
    meshGears() ||

    ensureHomedInXY() ||
    homeZ(tool) || // home Z so we can enter the xy pos with decent precision
    centerTool() ||
    homeZ(tool) || // re-home Z at a _slightly_ different XY (we've seen a 30um differnce in the measurement)

    raise()
  );
}

int Dispenser::unprepare() {
  setHomedState(Z_AXIS, 0);
  return (
    setDispenseHeight(0.0f)
  );
}

int Dispenser::asyncMove(float x, float y, float z, float e, float f, bool applyDispenseHeight) {
  if (applyDispenseHeight) {
    z += m_dispenseHeight;
  }
  return asyncMove(x, y, z, e, f);
}

float Dispenser::dispenseHeight() {
  return m_dispenseHeight;
}

int Dispenser::setDispenseHeight(float height) {
  if (height < 0.0f || height > 2.0f ) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set dispensing height to "); SERIAL_ERROR(height);
    SERIAL_ERRORPGM("mm, value is outside expected range\n");
    return -1;
  }

  // Set the height
  m_dispenseHeight = height;

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_PAIR("Dispense height set to ", m_dispenseHeight);
    SERIAL_ECHOPGM("mm");
    SERIAL_EOL;
  }

  return 0;
}

}