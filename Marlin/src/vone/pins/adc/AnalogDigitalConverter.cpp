#include "AnalogDigitalConverter.h"

#include "../BedTemperaturePin/BedTemperaturePin.h"
#include "../PTopPin/PTopPin.h"

#include "../../../../Marlin.h"
// #include "temperature.h"
// #include "watchdog.h"
#include "../../../../macros.h"
// #include "src/api/api.h" // Router::RampUpDuration
// #include "src/utils/rawToVoltage.h"

#include <avr/io.h>

namespace adc
{

static void startRead(byte pin) {
  // ADCSRB = 0; TODO: confirm not needed

  ADMUX = (
    _BV(REFS0) |  // use internal voltage refernece (AVcc)
    (pin & 0x07)  // mask off all but the last 3 bits
  );

  // Start Conversion
  SBI(ADCSRA, ADSC);
}

AnalogDigitalConverter::AnalogDigitalConverter(
  PTopPin& ptopPin,
  BedTemperaturePin& bedTemperaturePin
)
  : _ptopPin(ptopPin)
  , _bedTemperaturePin(bedTemperaturePin)
{
  ADCSRA = (
    1<<ADEN | // Enable the ADC
    // 1<<ADSC | // Start Conversion -- TODO needed?
    1<<ADIF | // Clear the Interrupt Flag, prevents any pending calls to ADC_vect
    1<<ADPS0 | 1<<ADPS1 | 1<<ADPS2 // use prescale of 128, gives us ~104us per reading
  );
  ADCSRB = 0;

  // TODO: will the isr be called eventually if isr has not be enabled yet?
  startRead(_bedTemperaturePin.analogPin());
}

void AnalogDigitalConverter::isr() {
  switch(state) {

    case StartBedTemp:
      startRead(_bedTemperaturePin.analogPin());
      state = MeasureBedTemp;
      break;
    case MeasureBedTemp:
      _bedTemperaturePin.addAdcSample(ADC);
      state = StartPTop;   //TODO: pretty sure we can prep here, not need to wait for next cycle
      break;

    case StartPTop:
      if (_ptopPin.allowAdcReads()) {
        startRead(_ptopPin.analogPin());
      }
      state = MeasurePTop;
      break;
    case MeasurePTop:
      _ptopPin.addAdcSample(ADC);
      state = StartBedTemp;
      break;

    case StartupDelay:
      state = StartBedTemp;
      break;
  }
}


// Why have periodic sampling ?
//  -- blocking analog reads would be fine for temp management
//  -- router detachment detection?

// router
//   - hold low for 500ms on boot/reset (or same length as ack)
//   - toggle low for Xms to ack
//   - detect disconnect as fast as possible

// FW
//  - ADC
//    - check voltage every 250s (or 50ms)
//    - check temp every 250s
//          - might need to average over up to 16 samples
//    - read new value probing
//    -


// ADC
//  option 1) read each pin as fast as we can (as fast as we do now)
//    then a) use direct analog reads + a flag to temporarily disable auto reads
//      or
//         b) use the auto read values, blocking if needed *** waiting for the next
//            value should be fine. if not
//              - make temp reads less frequent when heat below 40ish
//              - add code to disable ADC (like now)
//              - add code to compare time or movement to sample time
//                cuz it's probably already more recent
//
//  option 2) auto read less often, read directly when requested ()
//
// Why 16 samples?
//




// roll the zero (or threshold counting into this class, cuz detecting bad readings
// kinda seems ok/useful for the temp sensor...at least tolerable and cleaner than
// the current mess).
// ... may not need to roll it in depending on the interface between ADC and PTopPin
// (i.e. if we register an add method, then we can include zero counting ... downside would be
// maybe more complexity?)

// inline AdcSampledValue into Helper, since currentValue doesn't need add or reset
// only time ADC interupt thread will bump into main thread is if it's reading currentValue
// which should be 1) rare 2) quick

// does ADC own Helper or does PTopPin ?
//  needs to be PTopPin, since it's mode decides if the sample is added or

// ADC needs pin number, and allowADCRead() ...

}
