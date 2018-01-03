#include "NullTool.h"
#include "Probe.h"
#include "Dispenser.h"
#include "Router.h"

namespace tools {

// Facilitates permanent preallocate each tool
// Note: Deallocation a tool at runtime could leave the main loop's
//       thread in a deleted object and preventing that would increase complexity
class ToolBox {
  public:
    NullTool nullTool;
    Probe probe;
    Dispenser dispenser;
    Router router;

    ToolBox(Stepper& stepper, PTopPin& pin)
      : nullTool(stepper)
      , probe(stepper, pin)
      , dispenser(stepper)
      , router(stepper, pin)
    {
      setTool(&nullTool);
    }

    Tool& currentTool() { return *m_currentTool; }
    void setTool(Tool* tool);

  private:
    Tool* volatile m_currentTool = nullptr;
};

}
