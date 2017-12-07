// #include "ToolDetector.h"
// #include "../../../Marlin.h"
// #include "../../api/api.h"
// #include "../pins/PTopPin/PTopPin.h"
//
//
// const char* toolTypeAsString(Tool tool) {
//   switch(tool) {
//     case TOOLS_NONE: return "None";
//     case TOOLS_PROBE: return "Probe";
//     case TOOLS_DISPENSER: return "Dispenser";
//     case TOOLS_ROUTER: return "Router";
//     default: return "unknown";
//   }
// }
//
// static Tool s_toTool(Tool tool, enum ToolStates state) {
//   switch (state) {
//     case TOOL_STATE_PROBE_MOUNTED:
//     case TOOL_STATE_TRIGGERED:
//       return TOOLS_PROBE;
//
//     case TOOL_STATE_ROUTER_MOUNTED:
//       return TOOLS_ROUTER;
//
//     case TOOL_STATE_NOT_MOUNTED:
//       return tool == TOOLS_DISPENSER ? TOOLS_DISPENSER : TOOLS_NONE;
//
//     default:
//       return TOOLS_NONE;
//   }
// }
//
// ToolDetector::ToolDetector(PTopPin& pin)
//   : _ptopPin(pin)
//   , _tool(ToolType::Unknown)
// {
//   setTool(ToolType::None);
// }
//
// void ToolDetector::sendToolStatusUpdate() {
//   SERIAL_PROTOCOLPGM("toolUpdate");
//   SERIAL_PROTOCOLPGM(" type:"); SERIAL_PROTOCOL(toolTypeAsString(tool));
//   SERIAL_PROTOCOLPGM("\n");
// }
//
// void ToolDetector::setTool(ToolType type) {
//   if (_tool.type == type) {
//     return;
//   }
//   SERIAL_ECHO_START;
//   SERIAL_ECHOPGM("Swapping "); SERIAL_ECHO(toolTypeAsString(_tool.type));
//   SERIAL_ECHOPGM(" for "); SERIAL_ECHOLN(toolTypeAsString(type));
//   resetToolPreparations();
//   _tool = createTool(type);
//   sendToolStatusUpdate();
// }
//
// // Returns true if the most recent voltage reading matches the given tool
// bool ToolDetector::toolIsStill(Tool tool) {
//   const auto mostRecentVoltage = _ptopPin.voltage();
//   return tool == s_toTool(tool, classifyVoltage(tool, mostRecentVoltage));
// }
//
// void ToolDetector::periodicWork() {
//   // Run periodically
//   static unsigned long nextCheckAt = 0;
//   const auto now = millis();
//   if (now < nextCheckAt) {
//     return;
//   }
//   nextCheckAt = now + 1000;
//
//   // Note: we check this after the run-periodically code so that
//   // enabling/disabling does not impact the timing of tool change
//   // checks relative other tasks -- not critical, but it makes for
//   // a more consistent/predictable system
//   if (!_enabled) {
//     return;
//   }
//
//   // TODO: check ptopPin for intermittent disconnects
//   // ptopPin.sawDetach() ???
//   // ptopPin.clearDetach() ???
//
//   // Return if no change
//   auto currentTool = getTool();
//   if (toolIsStill(currentTool)) {
//     return;
//   }
//
//   // Determine the tool
//   // Note: determineToolState is slow becuase
//   // it takes time for the voltage to settle
//   const auto newTool = s_toTool(tool, determineToolState(tool));
//   setTool(newTool);
// }
