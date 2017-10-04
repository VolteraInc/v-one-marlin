#pragma once

#include "../../../Marlin.h"
#include <avr/io.h>

class HeaterPin {
  public:
    inline HeaterPin(int pin);
    inline ~HeaterPin();

    inline bool isHeating() const;

    inline void startHeating();
    inline void stopHeating();

  private:
    int _pin;
    bool _isHeating = false;
};

HeaterPin::HeaterPin(int pin): _pin(pin) {
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
}

HeaterPin::~HeaterPin() {
  stopHeating();
}

bool HeaterPin::isHeating() const {
  return _isHeating;
}

void HeaterPin::startHeating() {
  digitalWrite(_pin, HIGH);
  _isHeating = true;
}

void HeaterPin::stopHeating() {
  digitalWrite(_pin, LOW);
  _isHeating = false;
}
