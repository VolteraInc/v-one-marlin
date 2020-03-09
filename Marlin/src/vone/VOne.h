#pragma once

#include "../utils/Reporters.h"

#include "pins/PinSet.h"
#include "endstops/Endstops.h"
#include "endstops/EndstopMonitor.h"
#include "pins/adc/AnalogDigitalConverter.h"
#include "bed/heater/Heater.h"
#include "stepper/Stepper.h"

#include "tools/ToolBox.h"
#include "toolDetection/ToolDetector.h"

#include "motors/Motors.h"

// DEFER: ideally these would live in the cpp file,
//        but slicing up stepper_isr into h-files is
//        too much work for right now
#define ENABLE_TEMPERATURE_INTERRUPT()  SBI(TIMSK0, OCIE0B)
#define DISABLE_TEMPERATURE_INTERRUPT() CBI(TIMSK0, OCIE0B)
#define TEMPERATURE_ISR_ENABLED()      TEST(TIMSK0, OCIE0B)

class VOne {
  public:
    VOne(
      int ptopDigitalPin,
      int ptopAnalogPin,
      int bedTemperatureAnalogPin,
      int heaterDigitalPin,
      ZSwitch::Type zSwitchType
    );

    PinSet pins;
    Endstops endstops;
    adc::AnalogDigitalConverter adc;
    Heater heater;
    Motors motors;

  private:
    EndstopMonitor m_endstopMonitor;

  public:
    Stepper stepper;

    tools::ToolBox toolBox;
    toolDetection::ToolDetector toolDetector;

    void start();
    void frequentInterruptibleWork();
    void outputStatus();
    void periodicReport();

  private:
    unsigned long m_nextStatsCheckAt = 0;
    LowWaterReporter m_memoryUsage;

    void updateStats();

};
