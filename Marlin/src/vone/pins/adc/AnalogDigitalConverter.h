#pragma once

class BedTemperaturePin;
class PTopPin;

namespace adc
{

// All use of the ADC, including calls to analogRead, should go through
// this class, otherwise we will see invalid values due to overlapping use
// of the ADC.
//
// Great references on ADC:
//   https://www.gammon.com.au/adc
//   https://sites.google.com/site/qeewiki/books/avr-guide/analog-input
//   http://www.atmel.com/Images/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
//
// Design Notes:
//   Why use periodic interrupts to read ADC value ?
//     We want to know that temperature changes and tool detaches are not missed
//   Why not use automatic triggering (ADATE) ?
//     It was not clear when it was safe to change the MUX and know that the
//     next interrupt would provide data from that source.
//   Why do the timestamps in the sample drift / why not use a timer ?
//     We explicitly triggering convesions at end of _isr() to the timing will
//     on the code path. Explicit triggering gives us cleaner encapsulization
//     of the ADC because we don't need to configure a timer is setup().
//   Why not allow direct analog reads ?
//     it would disrupt the regularity of sampling, not clear if we could miss
//     an intermittent tool. Retrieving the most recent, or next, sample seem
//     sufficent
//
class AnalogDigitalConverter {
  public:
    AnalogDigitalConverter(
      PTopPin& ptopPin,
      BedTemperaturePin& bedTemperaturePin
    );

    inline void _isr() volatile;
  private:
    PTopPin& _ptopPin;
    BedTemperaturePin& _bedTemperaturePin;

    enum class State: int {
      ReceivePTop,
      ReceiveBedTemp
    };
    State state = State::ReceiveBedTemp;
};

}
