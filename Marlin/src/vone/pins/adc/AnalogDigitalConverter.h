#pragma once

// #include <avr/interrupt.h>
#include <avr/io.h>

#include "../BedTemperaturePin/BedTemperaturePin.h"
#include "../PTopPin/PTopPin.h"

#include "../../../../Marlin.h"
#include "../../../../macros.h"

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
//   How often do we sample ?
// TODO: comment here on frequency and in PTopPin and BedTemperaturePin near sample count
// TODO: reduce number of samples in BedTemperaturePin ?
// One we collect enough samples, report values and reset
// Note: Samples accumulate every 66ms (measured).
// This matches the expected frequency: 4 states * 16 samples * 1/(16000000/64/256) ~= 66ms
//
// Note: changing a form OUTPUT to INPUT generates noise for all analog reads
// that's why we've encapsulate this code here

namespace adc
{

class AnalogDigitalConverter {
  public:
    inline AnalogDigitalConverter(
      PTopPin& ptopPin,
      BedTemperaturePin& bedTemperaturePin
    );

    inline void frequentInterruptibleWork();

  private:
    enum class State : int {
      ReceivePTop,
      ReceiveBedTemp
    };

    PTopPin& _ptopPin;
    BedTemperaturePin& _bedTemperaturePin;
    State state;

    inline int getAdcPin();
    inline void startRead(byte pin);
};


void AnalogDigitalConverter::startRead(byte pin) {
  // Set source pin
  ADMUX = (
    _BV(REFS0) | // use internal voltage refernece (AVcc)
    pin
  );

  // Start conversion
  SBI(ADCSRA, ADSC);
}

AnalogDigitalConverter::AnalogDigitalConverter(
  PTopPin& ptopPin,
  BedTemperaturePin& bedTemperaturePin
)
  : _ptopPin(ptopPin)
  , _bedTemperaturePin(bedTemperaturePin)
{
  // Enable the ADC
  SBI(ADCSRA, ADEN);

  // Use prescale of 128 which gives us approximately 104us per reading
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);

  // Start first conversion
  // Note: must match initial state
  state = State::ReceiveBedTemp;
  startRead(_bedTemperaturePin.analogPin());
}

int AnalogDigitalConverter::getAdcPin() {
  return ADMUX & 0x7;
}

// Rather than waiting for reads to complete we start the conversion
// then pick up the result the next time this isr() is called.
void AnalogDigitalConverter::frequentInterruptibleWork() {
  long adcValue = ADC;
  auto targetPin = 0;

  switch(state) {
    case State::ReceivePTop:
      // If we read the p-top pin add it
      if (getAdcPin() == _ptopPin.analogPin()) {
        _ptopPin.addAdcSample(adcValue);
      }

      // prepare to read bed temp
      targetPin = _bedTemperaturePin.analogPin();
      state = State::ReceiveBedTemp;
      break;

    case State::ReceiveBedTemp:
      _bedTemperaturePin.addAdcSample(adcValue);

      // Sometimes p-top is in use elsewhere, so we skip reading it
      // Note: We could start a read of bed temp but have decided not to
      //       becuase it would make the sampling period inconsistent
      //       and unnecessarily dependent on the state of ptop.
      if (_ptopPin.tryStartAdcSampling()) {
        targetPin = _ptopPin.analogPin();
      }
      state = State::ReceivePTop;
      break;
  }

  // Start the next conversion
  // Note: we use an explicit 'start' so that we can change the targetPin and
  //       know that the next value seen comes from that pin. Other options like
  //       automatic triggering (ADATE) make it difficult to guaranteed this.
  //         http://www.atmel.com/Images/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
  startRead(targetPin);
}

}
