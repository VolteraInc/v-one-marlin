#include "NullTool.h"
#include "../../../Marlin.h"
#include "../../api/api.h"

tools::NullTool::NullTool(Stepper& stepper)
  : tools::Tool(stepper)
{
}

int tools::NullTool::prepareToMoveImpl() {
  return ensureHomedInXY(*this);
}

int tools::NullTool::resetPreparationsImpl() {
  setHomedState(Z_AXIS, 0);
  return 0;
}

int tools::NullTool::enqueueMove(float x, float y, float z, float e, float f) {
  // TODO: should pass m_stepper or call a method on m_stepper
  //       but some refactoring is needed to do that.
  return asyncRawMove(x, y, z, e, f);
}
