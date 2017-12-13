#include "NullTool.h"
#include "Probe.h"
#include "Dispenser.h"
#include "Router.h"

namespace tools {

// Facilitates permanent preallocate each tool
// Note: Deallocation a tool at runtime could leave the main loop's
//       thread in a deleted object and preventing that would increase complexity
struct ToolBox {
  NullTool nullTool;
  Probe probe;
  Dispenser dispenser;
  Router router;
  ToolBox(PTopPin& pin)
    : probe(pin)
    , router(pin)
  {
  }
};
