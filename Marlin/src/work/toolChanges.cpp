#include "../../Marlin.h"
#include "../api/api.h"

void toolChanges() {
  static auto nextCheckAt = millis();

  const auto now = millis();
  if (now >= nextCheckAt) {
    // Schedule next check
    nextCheckAt += 200;

    // Update the tool if necessary
    const auto tool = getTool();
    const auto newTool = determineMountedTool(tool);
    if (newTool != tool) {
      setTool(newTool);
    }
  }
}
