#pragma once

#include "Endstop.h"

class PTopPin;

struct Endstops {
  const Endstop xMin;
  const Endstop yMin;
  const Endstop zMax;
  const Endstop zSwitch;

  const Endstop xyPositionerLeft;
  const Endstop xyPositionerRight;
  const Endstop xyPositionerBack;
  const Endstop xyPositionerForward;
  const Endstop calibrationPlate;

  const Endstop toolSwitch;

  Endstops();

  const Endstop* lookup(const int pin) const;

  void outputStatus() const;
  void deprecated_outputStatus() const;
};
