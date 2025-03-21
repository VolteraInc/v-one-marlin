#pragma once

#include "Endstop.h"
#include "ZSwitch.h"


class PTopPin;

class Endstops {
  public:
    const Endstop xMin;
    const Endstop yMin;
    const Endstop zMax;

    const Endstop calibrationPlate;
    const Endstop toolSwitch;

    const ZSwitch zSwitch;
    const Endstop xyPositionerLeft;
    const Endstop xyPositionerRight;
    const Endstop xyPositionerBack;
    const Endstop xyPositionerForward;

    Endstops(ZSwitch::Type zSwitchType);

    #ifdef TRINAMIC_MOTORS
    //previously xLim and yLim
    const Endstop xMax;
    const Endstop yMax;
    #endif

    const Endstop* lookup(const int pin) const;
    void outputStatus() const;
    void reportChanges();
    void deprecated_outputStatus() const;

  private:
    struct {
      bool xMinTriggered = false;
      bool yMinTriggered = false;
      bool zMaxTriggered = false;

      bool calibrationPlateTriggered = false;
      bool toolSwitchTriggered = false;

      bool zSwitchTriggered = false;
      bool xyPositionerLeftTriggered = false;
      bool xyPositionerRightTriggered = false;
      bool xyPositionerBackTriggered = false;
      bool xyPositionerForwardTriggered = false;

      #ifdef TRINAMIC_MOTORS
      bool xMaxTriggered = false;
      bool yMaxTriggered = false;
      #endif
      
    } m_reportedStatus;
};
