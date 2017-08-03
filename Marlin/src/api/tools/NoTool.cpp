#include "../../../Marlin.h"
#include "../api.h"
#include "internal.h"

int NoTool::prepare(Tool) {
  return (
    ensureHomedInXY()
  );
}

int NoTool::unprepare(Tool) {
  setHomedState(Z_AXIS, 0);
  return 0;
}
