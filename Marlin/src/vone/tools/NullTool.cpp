#include "NullTool.h"
#include "../../api/api.h"

bool NullTool::attached() {
  return 
}

int NullTool::prepare() {
  return (
    ensureHomedInXY()
  );
}

int NullTool::unprepare() {
  setHomedState(Z_AXIS, 0);
  return 0;
}
