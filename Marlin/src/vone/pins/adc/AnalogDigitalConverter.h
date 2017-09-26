#pragma once

class BedTemperaturePin;
class PTopPin;

namespace adc
{

// All use of the ADC including calls to analogRead should use this
// class, otherwise we will see occasional invalid values due to
// overlapping use of the ADC
//
// Great reference on ADC:
//   https://www.gammon.com.au/adc
//
class AnalogDigitalConverter {
  public:
    AnalogDigitalConverter(
      PTopPin& ptopPin,
      BedTemperaturePin& bedTemperaturePin
    );

    void isr(); ///TODO: make private and use ADC_vect?

  private:
    PTopPin& _ptopPin;
    BedTemperaturePin& _bedTemperaturePin;

    // Rather than waiting for reads to complete we start them then check
    // ...
    enum AdcReadState {
      StartPTop,
      MeasurePTop,
      StartBedTemp,
      MeasureBedTemp,

      // TODO: necessary? this is a 104us delay
      StartupDelay, // Startup, delay initial reading a tiny bit so the hardware can settle
      // TemporarilyDisabled
    } state = StartupDelay;
};

}
