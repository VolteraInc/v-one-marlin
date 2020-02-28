#pragma once

#include "Endstop.h"
#include "ZSwitch.h"

class PTopPin;

struct Endstops {
  const Endstop xMin;
  const Endstop yMin;
  const Endstop zMax;
  const ZSwitch zSwitch;

  const Endstop xyPositionerLeft;
  const Endstop xyPositionerRight;
  const Endstop xyPositionerBack;
  const Endstop xyPositionerForward;
  const Endstop calibrationPlate;

  const Endstop toolSwitch;

  #ifdef TRINAMIC_MOTORS
  const Endstop xLim;
  const Endstop yLim;
  #endif

  Endstops(ZSwitch::Type zSwitchType);

  const Endstop* lookup(const int pin) const;

  void outputStatus() const;
  void deprecated_outputStatus() const;
};
