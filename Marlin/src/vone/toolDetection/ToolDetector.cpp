#include "ToolDetector.h"

#include "../../../Marlin.h"

namespace toolDection
{

ToolDetector::ToolDetector(ToolBox& tools, const PTopPin& pin)
  : m_tools(tools)
  , m_pin(pin)
{
  setTool(&m_tools.nullTool);
  m_detectedTool = m_currentTool;
}

void ToolDetector::setTool(Tool* newTool) {
  // Prevent interleaving calls to this function
  // (e.g from vCode and ToolDetector::frequentInterruptibleWork())
  ScopedInterruptDisable sid;

  if (m_currentTool == newTool) {
    return;
  }

  if (m_currentTool) {
    m_currentTool->deactivate();
  }

  m_currentTool = newTool;
  m_currentTool->activate();
}

void ToolDetector::updateTool() {
  // Keep current tool up-to-date
  setTool(m_detectedTool);
}

void ToolDetector::outputToolStatus() {
  const auto toolState = determineToolState(s_tool);
  SERIAL_ECHOPGM("Tool"); SERIAL_EOL;
  SERIAL_PAIR("  type: ", toolDetector.currentTool().typeName()); SERIAL_EOL;
  SERIAL_PAIR("  prepared: ", tool->preparedToMove()); ; SERIAL_EOL;

  SERIAL_ECHOPGM("Probe"); SERIAL_EOL;
  SERIAL_PAIR("  displacement: ", probe.displacement());
  SERIAL_EOL;

  SERIAL_ECHOPGM("Dispenser"); SERIAL_EOL;
  SERIAL_PAIR("  dispense height: ", dispenser.dispenseHeight());
  SERIAL_EOL;

  SERIAL_ECHOPGM("Router"); SERIAL_EOL;
  SERIAL_PAIR("  Speed: ", router.rotationSpeed());
  SERIAL_EOL;

  SERIAL_ECHOPGM("Homing"); SERIAL_EOL;
  SERIAL_PAIR("  x: ", getHomedState(X_AXIS)); SERIAL_EOL;
  SERIAL_PAIR("  y: ", getHomedState(Y_AXIS)); SERIAL_EOL;
  SERIAL_PAIR("  z: ", getHomedState(Z_AXIS)); SERIAL_EOL;
}

}