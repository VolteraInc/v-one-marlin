#include "CustomSerial.h"

#include "../../../../Marlin.h"

// baudrate of 300 is based on the rise and fall times of the capacitor on ptop (600-800 us)
static const int baud = 300;
static const int dummy_pin = A3;

CustomSerial::CustomSerial(int pin)
  : _pin(pin)
  , serial(dummy_pin, pin)
{
}

void CustomSerial::_send(char* msg) {
  SERIAL_ECHO_START;
  SERIAL_ECHO("Sending "); SERIAL_ECHOLN(msg);

  pinMode(_pin, OUTPUT);
  serial.begin(baud);
  serial.listen();
  serial.println(msg);
  timeSent = millis();
  serial.end();
}

void CustomSerial::_recvAcknowmedgement(bool& acknowledged) {
  pinMode(_pin, INPUT);

  // We must wait the full duration, otherwise the router will still be signalling
  // on this pin when we return to normalMode.
  const unsigned long tryUntil = timeSent + 300; //ms  //TODO: don't spend all 300ms here, do some reading then wait out the window later
  auto count = 0u;
  while (millis() <= tryUntil) {
    count += !digitalRead(_pin);
    delay(10);
  }

  // Must place pin back in output mode otherwise
  // tool will see it as a dismount
  // TODO: more the retry logic to ptopPin and return the count
  // then we can handle the pinMode stuff out there too
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);

  acknowledged = count > 3;

  SERIAL_ECHO_START;
  SERIAL_PAIR("Received ", acknowledged ? "ack" : "noAck");
  SERIAL_PAIR(", value = ", count);
  SERIAL_EOL;

  //delayUntil(timeSent + 300);
}

int CustomSerial::send(char* msg, unsigned maxAttempts, unsigned* o_numAttempts) {
  auto attempt = 1u;
  bool acknowledged = false;
  for (attempt = 1; attempt <= maxAttempts; ++attempt) {

    if (attempt > 1) {
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Retrying");
    }

    _send(msg);
    _recvAcknowmedgement(acknowledged);
    if (acknowledged) {
      break;
    }
  }

  if (o_numAttempts) {
    *o_numAttempts = attempt;
  }

  if (!acknowledged) {
    return -1;
  }

  return 0;
}
