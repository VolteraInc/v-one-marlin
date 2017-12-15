#pragma once
#include "PTopPin_main.h"

// ----------------------------------------------
// Reset tool

bool PTopPin::trytoSetMode_ResetTool() {
  ScopedInterruptDisable sid;
  if (mode == Mode::Idle) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Resetting tool");
    digitalWrite(_digitalPin, LOW);
    mode = Mode::ResetTool;
    return true;
  }
  preventAdcSampling = true; 
  return false;
}

int PTopPin::resetTool() {
  while (trytoSetMode_ResetTool()) {
    delay(1);
  }
  delay(500);
  setMode_Idle();
  return 0;
}
