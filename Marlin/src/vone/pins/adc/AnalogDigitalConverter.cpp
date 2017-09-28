#include "AnalogDigitalConverter.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "../BedTemperaturePin/BedTemperaturePin.h"
#include "../PTopPin/PTopPin.h"

#include "../../../../Marlin.h"
#include "../../../../macros.h"

#include <avr/io.h>

static volatile adc::AnalogDigitalConverter* s_adc = nullptr;

static void startRead(byte pin) {
  // Set source pin
  ADMUX = (
    _BV(REFS0) | // use internal voltage refernece (AVcc)
    pin
  );

  // Start conversion
  SBI(ADCSRA, ADSC);
}

static int getAdcPin() {
  return ADMUX & 0x7;
}

namespace adc
{

AnalogDigitalConverter::AnalogDigitalConverter(
  PTopPin& ptopPin,
  BedTemperaturePin& bedTemperaturePin
)
  : _ptopPin(ptopPin)
  , _bedTemperaturePin(bedTemperaturePin)
{
  // Initialize the pointer used in ADC_vect
  s_adc = this;

  // Enable the ADC
  SBI(ADCSRA, ADEN);

  // Use prescale of 128 which gives us approximately 104us per reading
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);

  // Enable Interrupts
  // i.e. call ADC_vect when results are ready
  SBI(ADCSRA, ADIE);

  // Start first conversion
  startRead(_bedTemperaturePin.analogPin());
}

// Rather than waiting for reads to complete we start the conversion
// then pick up the result the next time this isr() is called.
void AnalogDigitalConverter::_isr() volatile {
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
  // Note: we use an explicit 'start' so that was can change the targetPin and
  //       know that the next value came from that pin. Other options like automatic
  //       triggering (ADATE) make it difficult to guaranteed this.
  //         http://www.atmel.com/Images/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
  startRead(targetPin);
}

}

ISR(ADC_vect) {
  s_adc->_isr();
}
