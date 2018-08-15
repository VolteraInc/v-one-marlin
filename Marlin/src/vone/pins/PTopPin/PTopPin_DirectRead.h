#pragma once
#include "PTopPin_main.h"

// ----------------------------------------------
// Direct Read
// Note: Inlined for use my stepper. The stepper only needs digital reads, but
//       I'd rather keep all the reads together.

bool PTopPin::trytoSetMode_DirectRead() {
  ScopedInterruptDisable sid;
  if (mode == Mode::Idle) {
    mode = Mode::DirectRead;
    pinMode(_digitalPin, INPUT);
    return true;
  }
  preventAdcSampling = true;
  return false;
}

bool PTopPin::_readDigitalValue() {
  return digitalRead(_digitalPin) ^ P_TOP_ENDSTOP_INVERTING;
}

int PTopPin::readDigitalValue(bool& value) {
  ScopedInterruptDisable sid;
  if (mode == Mode::AdcSamplng) {
    // AdcSampling mode implies that the pin is already configured for INPUT
    // so we can read and return. It's important not to change the pin mode
    // (to OUTPUT), which would happen if we set the state to IDLE
    value = _readDigitalValue();
    return 0;

  } else if (trytoSetMode_DirectRead()) {
    // Mode was Idle, now it's DirectRead, which sets the pin mode to INPUT.
    // Return the mode to Idle before returning
    value = _readDigitalValue();
    setMode_Idle();
    return 0;

  } else {
    // Digital reads while in other modes would break things (e.g. Communication).
    // Preventing this is outside the scope of this class.
    logError
      << F("Unable to read digital value of p-top pin, ") << modeToString(mode)
      << F(" mode (") << (int)mode
      << F(") does not support digital reads")
      << endl;
    return -1;
  }
}
