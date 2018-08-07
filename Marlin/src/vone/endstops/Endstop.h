#pragma once

class __FlashStringHelper;

struct Endstop {
  const __FlashStringHelper* name;
  int pin;
  AxisEnum axis;
  int direction;
  bool isAxisLimit;

  Endstop(
    const __FlashStringHelper* name_,
    int pin_,
    AxisEnum axis_,
    int direction_,
    bool isAxisLimit_ = false
  ) : name(name_)
    , pin(pin_)
    , axis(axis_)
    , direction(direction_)
    , isAxisLimit(isAxisLimit_)
  {
    pinMode(pin, INPUT);
  }
};
