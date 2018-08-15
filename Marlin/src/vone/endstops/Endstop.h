#pragma once

#include "../../../Axis.h"
#include "../../../MarlinConfig.h"

class __FlashStringHelper;

struct Endstop {
  const __FlashStringHelper* name;
  const int pin;
  const AxisEnum axis;
  const int direction;
  const bool inverted;
  const bool isAxisLimit;

  Endstop(
    const __FlashStringHelper* name,
    int pin,
    AxisEnum axis,
    int direction,
    bool inverted,
    bool isAxisLimit = false
  ) : name(name)
    , pin(pin)
    , axis(axis)
    , direction(direction)
    , inverted(inverted)
    , isAxisLimit(isAxisLimit)
  {
    pinMode(pin, INPUT);
  }

  bool readTriggered() const {
    return (digitalRead(pin) == 1) ^ inverted;
  }
};
