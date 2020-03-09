#pragma once

#include "Endstop.h"
#include "ZSwitch.h"

class PTopPin;

class Endstops {
  public:
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
    void reportChanges();
    void deprecated_outputStatus() const;

  private:
    struct {
      bool xMinTriggered = false;
      bool yMinTriggered = false;
      bool zMaxTriggered = false;
      bool zSwitchTriggered = false;

      bool xyPositionerLeftTriggered = false;
      bool xyPositionerRightTriggered = false;
      bool xyPositionerBackTriggered = false;
      bool xyPositionerForwardTriggered = false;
      bool calibrationPlateTriggered = false;

      bool toolSwitchTriggered = false;
    } m_reportedStatus;
};
