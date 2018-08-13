#pragma once

#include "pins/PinSet.h"
#include "endstops/Endstops.h"
#include "endstops/EndstopMonitor.h"
#include "pins/adc/AnalogDigitalConverter.h"
#include "bed/heater/Heater.h"
#include "stepper/Stepper.h"

#include "tools/ToolBox.h"
#include "toolDetection/ToolDetector.h"

class VOne {
  public:
    VOne(
      int ptopDigialPin,
      int ptopAnalogPin,
      int bedTemperatureAnalogPin,
      int heaterDigitalPin
    );

    PinSet pins;
    Endstops endstops;
    adc::AnalogDigitalConverter adc;
    Heater heater;

  private:
    EndstopMonitor m_endstopMonitor;

  public:
    Stepper stepper;

    tools::ToolBox toolBox;
    toolDetection::ToolDetector toolDetector;

    // Perform work that must happen frequently but can be
    // interrupted (briefly) by time critical work like
    // serial character reads and the stepper.
    // Note: work done in the main loop can be delayed delayed by
    //       10s, 30s or longer. It depends on how long it takes
    //       to process the current command.
    inline void frequentInterruptibleWork() {
      // Allow other interrupts
      CBI(TIMSK0, OCIE0B);
      sei();

      // Updated ADC values are needed by heater and toolDetector
      adc.frequentInterruptibleWork();

      // Heater management
      // Note: Delaying heater updates (even for a few seconds) could
      //       result in several degrees of overshoot
      // TODO: temperature profiles should be processed here
      //       otherwise the next step in the profile can be delayed
      //       by command processing
      heater.frequentInterruptibleWork();

      // Tool Detection
      // Note: Delaying tool detach detection (even for a few seconds) could
      //       result in damage, i.e. a tool crash, broken drill bit, etc
      // Note: Voltage readings will start around 0 because we hold
      //       voltage low on boot (to reset the attached tool).
      //       To avoid this weirdness we ignore voltage readings
      //       until after 1000ms. This value was determined by observtion,
      //       (500ms was not enough, 600ms was close). This value depends
      //       on how much work is being performed during system setup
      const auto now = millis();
      if (now > 1000) {
        toolDetector.frequentInterruptibleWork();
      }

      // Restore interrupt settings
      // DEFER: is this needed ?
      SBI(TIMSK0, OCIE0B);
    }

};
