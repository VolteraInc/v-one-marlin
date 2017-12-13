#include "../../../Marlin.h"
// #include "toolDetection.h"

namespace toolDetection {

VoltageType stabilizedVoltageType(PTopPin& pin) {
  // We use a counter to detect consecutive equivalent
  // classifications because averaging would add even more lag.
  const auto start = millis();
  const int maxIterations = 12;
  int count = 0;
  auto previousType = VoltageType::Unknown;
  float voltages[maxIterations];
  for (int i = 0; i < maxIterations; ++i) {
    voltages[i] = pin.readVoltage();
    auto type = classifyVoltage(voltages[i]);

    if (previousType == type) {
      ++count;

      // Return if we've collect enough matching voltage type classications
      if (count >= 4) {
        const auto stop = millis();
        const auto numIterations = i + 1;

        SERIAL_ECHO_START;
        SERIAL_PAIR("Determined tool state '", toString(type));
        SERIAL_PAIR("' after ", numIterations);
        SERIAL_PAIR(" iterations (", stop - start);
        SERIAL_ECHOPGM(" ms)");
        SERIAL_EOL;

        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("  voltages = [");
        serialArray(voltages, numIterations);
        SERIAL_ECHOPGM("]");
        SERIAL_EOL;

        return type;
      }
    } else {
      // type differs from previous, reset counter
      count = 1;
      previousType = type;
    }
  }

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Warning: Unable to determine tool's state, too much variation in readings.");
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("  voltages = [");
  serialArray(voltages, maxIterations);
  SERIAL_ECHOPGM("]");

  return VoltageType::Unknown;
}

}