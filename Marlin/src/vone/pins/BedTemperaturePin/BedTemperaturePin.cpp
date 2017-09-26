
#include "ThermistorTable.h"
#include "BedTemperaturePin.h"

#include "../../../../Marlin.h"

BedTemperaturePin::BedTemperaturePin(int analogPin)
  : _analogPin(analogPin)
  , adcSamples(numSamples)
{
  // Disable the digital read buffer on this pin
  // Note: We only do analog reads on TEMP_BED_PIN, so we can disable its
  // digital read buffer.
  //   "If you are doing only ADC conversions on some or all of the analog
  //    inputs, you should disable the digital buffers, to save power.
  //    Once disabled, a digitalRead on those pins will always read zero."
  //       - https://www.gammon.com.au/adc
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Disable digital reads of the bed temperature pin");
  DIDR0 |= 1 << _analogPin;
  SERIAL_ECHO_START;
  SERIAL_PAIR("DIDR0 is now ", DIDR0); SERIAL_EOL;
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)

// DEFER: this code could be significantly clearer
// could potentially move to utils too
float BedTemperaturePin::rawToTemperature(long raw) {
  float celsius;
  for (auto i = 1u; i < BEDTEMPTABLE_LEN; ++i) {
    if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw) {
      celsius = (
        PGM_RD_W(BEDTEMPTABLE[i-1][1]) +
          (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) *
          (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /
          (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]))
      );
      return celsius;
    }
  }

  // Overflow, use last value in table
  celsius = PGM_RD_W(BEDTEMPTABLE[BEDTEMPTABLE_LEN - 1][1]);
  return celsius;
}
