#pragma once

class __FlashStringHelper;

struct Endstop {
  const __FlashStringHelper* name;
  const int pin;
  const AxisEnum axis;
  const int direction;
  const bool inverted;
  const bool isAxisLimit;

  Endstop(
    const __FlashStringHelper* name_,
    int pin_,
    AxisEnum axis_,
    int direction_,
    bool inverted_,
    bool isAxisLimit_ = false
  ) : name(name_)
    , pin(pin_)
    , axis(axis_)
    , direction(direction_)
    , inverted(inverted_)
    , isAxisLimit(isAxisLimit_)
  {
    pinMode(pin, INPUT);
  }

  bool readTriggered() const {
    return digitalRead(pin) == 1 ^ inverted;
  }
};
