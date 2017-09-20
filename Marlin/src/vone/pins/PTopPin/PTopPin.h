#pragma once
// #include "../../../macros.h" // CRITICAL_SECTION

class PTopPin {
  public:
    PTopPin(int pin);

//     const AdcSampledValue& voltage();
//
//     // Direct reading
//     int readVoltages(float voltages[], size_t size, unsigned delayMs = 0);
//
//     // ADC sampling
//     bool allowAdcReads();
//     void addRawSample(long sample);
//
  private:
    enum class Mode {
      Idle,
      Communication,
      SoftDetachTool,
      AdcSamplng,
      DirectRead
    };

    int _pin;
//
//     // State management
//     volatile Mode mode = Mode::Idle;
//     void _setMode(enum Mode);
//
//     // ADC sampling
//     // TODO: Thermistor tables use 16, we should figure out a
//     //       good value to use here.
//     const unsigned AdcSamplesCount = 4;   use 1 ?
//     volatile long preventAdcSamplingUntil = 0;
//     volatile AdcSampledValue adcSamples;
//     volatile AdcSampledValue nextVoltage;
//     AdcSampledValue currentVoltage;
//
//     // Communication
//     SoftwareSerial serial;
//     long commandSentTime = 0;
//     int send(char* msg);
};

// // ----------------------------------------------
// // Inline implemtations for use by AdcSampler
// // The AdcSampler uses an interupt, which makes performance important.
// // That is if it runs too slowly we could end up delaying the stepper
// // So we implement the functions used my the AdcSampler in this header,
// // which allows the compiler to inline them. I have not measured performance
// // to comfirm the need for this, I just wanted to get rid of some macros
// // and improve the design of this code.
// // Note: the order of the functions also matters becuase the compiler needs
// // to see the implementation before the function is used (e.g. _setMode)
//
// bool PTopPin::allowAdcReads() {
//   return mode == Mode::AdcSamplng;
// }
//
// // note: should be called from within a critical section
// // unless setting to Idle
// int PTopPin::_setMode(PTopPin::Mode newMode) {
//   if (mode == newMode) {
//     return 0;
//   }
//
//   if (newMode == Mode::Idle ) {
//     SET_OUTPUT(_pin);
//     WRITE(pin, 1);
//   } else {
//     if (mode != Mode::Idle) {
//       SERIAL_ERROR_START;
//       ...
//       return -1;
//     }
//
//     switch (newMode) {
//       case Mode::SoftDetachTool:
//         SET_OUTPUT(_pin);
//         WRITE(_pin, 0);
//         break;
//
//       case Mode::AdcSamplng:
//         SET_INPUT(_pin);
//         adcSamples.reset();
//         break;
//
//       case Mode::DirectRead:
//         SET_INPUT(_pin);
//         break;
//
//       case Mode::Communication:
//         break;
//     }
//   }
//
//   mode = newMode;
// }
//
// void PTopPin::setModeIfIdle(PTopPin::Mode mode) {
//   CRITICAL_SECTION_START;
//   if (mode == Mode::Idle) {
//     _setMode(mode);
//   }
//   CRITICAL_SECTION_END;
// }
//
// int PTopPin::tryStartAdcSampling(long time) {
//   if (time < preventAdcSamplingUntil) {
//     return false;
//   }
//   return setModeIfIdle(Mode::AdcSamplng);
// }
//
// void PTopPin::addAdcSample(long time, long value) {
//   // adc sampling can be interrupted by other states
//   // if that happens we stop accepting samples.
//   // Note: the accumulated values will be reset
//   // the next time we enter Mode::AdcSamplng.
//   if (mode != Mode::AdcSamplng) {
//     return;
//   }
//
//   adcSamples.add(time, value);
//
//   if (adcSamples.isReady()) {
//     CRITICAL_SECTION_START;
//       nextVoltage = adcSamples;
//       adcSamples.reset();
//       preventAdcSamplingUntil = time + 10;
//       _setMode(Mode::Idle);
//     CRITICAL_SECTION_END;
//   }
// }
