#pragma once

#include "NullTool.h"
#include "Probe.h"
#include "Dispenser.h"
#include "SmartDispenser.h"
#include "Drill.h"

namespace tools {

// Facilitates permanent preallocate each tool
// Note: Deallocation a tool at runtime could leave the main loop's
//       thread in a deleted object and preventing that would increase complexity
class ToolBox {
  public:
    NullTool nullTool;
    Probe probe;
    Dispenser dispenser;
    SmartDispenser smartDispenser;
    Drill drill;

    ToolBox(Stepper& stepper, PTopPin& pin, const Endstop& toolSwitch)
      : nullTool(stepper)
      , probe(stepper, pin, toolSwitch)
      , dispenser(stepper)
      , smartDispenser(stepper)
      , drill(stepper, pin)
    {
      setTool(&nullTool);
    }

    Tool& currentTool() { return *m_currentTool; }
    void setTool(Tool* tool);

  private:
    Tool* volatile m_currentTool = nullptr;
};

}
