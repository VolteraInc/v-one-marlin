#include "../../planner.h"
#include "../../serial.h"

void reportBufferEmpty() {
  static uint8_t buffer_fill = 0;
  uint8_t new_buffer_fill = movesplanned();
  if (buffer_fill && !new_buffer_fill) {
    protocol << F("empty") << endl;
  }
  buffer_fill = new_buffer_fill;
}
