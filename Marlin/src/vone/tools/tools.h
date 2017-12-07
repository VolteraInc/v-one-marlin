#pragma once

namespace tools {
  enum class ToolType {
    Unknown,
    None,
    Probe,
    Dispenser,
    Router
  };

  void emplace(Tool* toolBuffer, ToolType type);
}
