#include "ToolBox.h"
#include "../../../serial.h"

static void outputToolUpdate(const char* type, unsigned int version) {
  protocol
    << F("toolUpdate")
    << F(" type:") << type
    << F(" version:") << version
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
  outputToolUpdate(m_currentTool->type(), m_currentTool->version());
}
