#pragma once

#include "Endstop.h"

class PTopPin;

struct Endstops {
  Endstop xMin;
  Endstop yMin;
  Endstop zMax;
  Endstop zSwitch;

  Endstop xyPositionerBack;
  Endstop xyPositionerLeft;
  Endstop xyPositionerRight;
  Endstop xyPositionerForward;
  Endstop calibrationPlate;

  PTopPin& toolSwitch;

  Endstops(PTopPin& toolSwitch);

  int outputStatus();
  int deprecated_outputStatus();

};