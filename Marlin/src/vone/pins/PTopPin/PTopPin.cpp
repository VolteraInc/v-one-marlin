
#include "PTopPin.h"
#include "../../../utils/rawToVoltage.h"


PTopPin::PTopPin(int pin)
  : _pin(pin)
  // , customSerial(pin)
{
//   // Start with p-top low for a few milliseconds so that
//   // if there is an attached tool, it will see a disconnect
//   SERIAL_ECHO_START;
//   SERIAL_ECHOLNPGM("Soft detaching tool (i.e. output 0)");
//   _setMode(Mode::SoftDetachTool);
//   delay(500);
//   _setMode(Mode::Idle);
}

// // float PTopPin::awaitNextVoltage() {
// //   voltage();
// //   SERIAL_ECHO_START;
// //   SERIAL_ECHOLN("Waiting for next voltage...");
// //   auto start = millis();
// //   while (!nextVoltage.isReady()) {
// //     delay(1);
// //   }
// //   auto duration = millis() - start;
// //   SERIAL_ECHO_START;
// //   SERIAL_PAIR("Waited ", duration); SERIAL_ECHOLNPGM("ms for next voltage reading");
// //   return voltage();
// // }
//
// We want to sample volatages to detect tool detachs.
// for tool classifiction we could just read directly.
// - if we are alrady reading, why not use it
// - direct read would get a bad value for the first read
// - it is going to take 60ms to classify regardless of the approach, because the cap has to charge
//
// Still need to detect detaches.
//  - need to remeber them (during sampling)
//  - need to handle them in ToolDetector
//
// float PTopPin::voltage() {
//   if (nextVoltage.isReady()) {
//     CRITICAL_SECTION_START;
//       currentVoltage = nextVoltage;
//       nextVoltage.reset();
//     CRITICAL_SECTION_END;
//   }
//   return currentVoltage.value();
// }
//
// int PTopPin::readVoltages(float voltages[], size_t size, unsigned delayMs) {
//   waitForIdleThenSetMode(Mode::DirectRead);
//
//   ADC.readMultiple(P_TOP_STATE_PIN, size, [&voltages, delayMs](int raw, unsigned idx) {
//     voltages[idx] = rawToVoltage(raw);
//     if (delayMs) {
//       delay(delayMs);
//     }
//     return 0;
//   });
//
//   _setMode(Mode::Idle);
// }
//
// // ----------------------------------------------
// // Communication Mode
//
// void PTopPin::waitForIdleThenSetMode(PTopPin::Mode mode) {
//   while (setModeIfIdle(mode)) {
//     delay(1);
//   }
// }
//
// int PTopPin::send(char* msg) {
//   waitForIdleThenSetMode(Mode::Communication);
//
//   const unsigned maxAttempts = 3;
//   unsigned numAttempts;
//   if (customSerial.send(msg, maxAttempts, numAttempts)) {
//     return -1;
//   }
//
//   if (numAttempts > 2) {
//     SERIAL_ECHO_START;
//     SERIAL_PAIR("NOTICE: p-top message sent on attempt ", numAttempts);
//     SERIAL_PAIR(" of ", maxAttempts);
//     SERIAL_EOL;
//   }
//
//   return return_value;
// }
