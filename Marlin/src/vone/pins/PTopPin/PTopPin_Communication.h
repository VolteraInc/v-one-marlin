#pragma once

#include "PTopPin_main.h"

// ----------------------------------------------
// Communication

bool PTopPin::trytoSetMode_Communication() {
  ScopedInterruptDisable sid;
  if (mode == Mode::Idle) {
    mode = Mode::Communication;
    return true;
  }
  return false;
}

int PTopPin::send(char* msg) {
  int returnValue = -1;

  while (!trytoSetMode_Communication()) {
    delay(1);
  }

  const unsigned maxAttempts = 3;
  unsigned numAttempts = 0;
  if (customSerial.send(msg, maxAttempts, &numAttempts)) {
    goto DONE;
  }

  returnValue = 0;

DONE:
  setMode_Idle();

  if (returnValue == 0 && numAttempts > 2) {
    SERIAL_ECHO_START;
    SERIAL_PAIR("NOTICE: p-top message sent on attempt ", numAttempts);
    SERIAL_PAIR(" of ", maxAttempts);
    SERIAL_EOL;
  }

  return returnValue;
}
