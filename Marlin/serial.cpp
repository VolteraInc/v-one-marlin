#include "serial.h"

bool logging_enabled = false;
const char endl[] = "\n";
bool logging::inISR = false;