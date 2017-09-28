
#include "PTopPin.h"

PTopPin::PTopPin(int digitalPin, int analogPin)
  : _digitalPin(digitalPin)
  , _analogPin(analogPin)
  , adcSamples(numSamples)
  // , customSerial(digitalPin)
{
  // Reset attached tool, if there is one
  resetTool();
}

const char* PTopPin::modeToString(PTopPin::Mode mode) {
  switch(mode) {
    case Mode::Idle: return "Idle";
    case Mode::Communication: return "Communication";
    case Mode::ResetTool: return "Reset Tool";
    case Mode::AdcSamplng: return "Analog Sampling";
    case Mode::DirectRead: return "Direct Read";
  }
  return "Unknown";
};


// ----------------------------------------------
// Reset tool

bool PTopPin::trytoSetMode_ResetTool() {
  ScopedInterruptDisable sid;
  if (mode == Mode::Idle) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Resetting tool");
    digitalWrite(_digitalPin, 0);
    mode = Mode::ResetTool;
    return true;
  }
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

// int PTopPin::send(char* msg) {
//   while (!trytoSetMode_Communication()) {
//     delay(1);
//   }
//
//  const unsigned maxAttempts = 3;
//  unsigned numAttempts = 0;
//  if (customSerial.send(msg, maxAttempts, numAttempts)) {
//    return -1;
//  }
//
//  if (numAttempts > 2) {
//    SERIAL_ECHO_START;
//    SERIAL_PAIR("NOTICE: p-top message sent on attempt ", numAttempts);
//    SERIAL_PAIR(" of ", maxAttempts);
//    SERIAL_EOL;
//  }
//
//   return 0;
// }
