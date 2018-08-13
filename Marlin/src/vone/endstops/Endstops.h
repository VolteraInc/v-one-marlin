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

  PTopPin& ptop;


  Endstops(PTopPin& ptop);
  int outputStatus() const;
  int deprecated_outputStatus() const;
};