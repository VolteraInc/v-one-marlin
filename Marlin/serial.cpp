#include "serial.h"

bool logging_enabled = false;
const char endl[] = "\n";

namespace logging {
  bool inISR = false;
  LogSuppesser suppressLog;
}
