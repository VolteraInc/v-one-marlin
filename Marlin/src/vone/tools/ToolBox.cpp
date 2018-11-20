#include "ToolBox.h"
#include "../../../serial.h"

static void outputToolUpdate(const char* type) {
  protocol
    << F("toolUpdate type:") << type
    << endl;
  log
    << F("toolUpdate type:") << type << F(" at ") << millis()
    << endl;
}

void tools::ToolBox::setTool(tools::Tool *tool) {
  if (m_currentTool == tool) {
    return;
  }

  if (m_currentTool) {
    m_currentTool->detach();
  }

  m_currentTool = tool;
  m_currentTool->attach();
  outputToolUpdate(m_currentTool->name());
}
