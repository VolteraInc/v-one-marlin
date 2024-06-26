#pragma once

#include "../../../Axis.h"
#include "../../../MarlinConfig.h"
//#Include "XYZEndstops.cpp"

class __FlashStringHelper;

struct Endstop {
  const __FlashStringHelper* name;
  const int pin;
  const AxisEnum axis;
  const int direction;
  const bool inverted;
  const bool virtualEndstop; //this is for endstops that do not trigger from pin state

  Endstop(
    const __FlashStringHelper* name,
    int pin,
    AxisEnum axis,
    int direction,
    bool inverted,
    bool virtualEndstop
  ) : name(name)
    , pin(pin)
    , axis(axis)
    , direction(direction)
    , inverted(inverted)
    , virtualEndstop(virtualEndstop)
  {
    if(!virtualEndstop)
    {
      pinMode(pin, INPUT);
    }
    
  }

  bool readTriggered() const {
    if(!virtualEndstop)
    {
      return (digitalRead(pin) == 1) ^ inverted;
    }
    else
    {
      //to-do, call to xyz function, pass reference to this endstop
      //return isXYZTouch(*this);
    }
  }
};
