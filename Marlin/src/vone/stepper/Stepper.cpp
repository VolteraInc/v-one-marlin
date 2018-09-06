#include "Stepper.h"

#include "../endstops/Endstop.h"
#include "../endstops/EndstopMonitor.h"

#include "../../../planner.h"
#include "../../../stepper.h"
#include "../../../serial.h"

Stepper::Stepper(
  EndstopMonitor& endstopMonitor
) : endstopMonitor(endstopMonitor)
  , maxStepsComplete(F("stepper isr finished first set of steps"), F("us"))
  , maxInterruptsAllowed(F("stepper isr allowing serial interrupts"), F("us"))
  , maxCompletedWithoutTriggers(F("stepper isr completed without triggering"), F("us"))
  , maxCompletedWithoutTriggersTics(F("stepper isr completed without triggering"), F("tics"), 200)
  , maxStepRate(F("stepper step rate"), F("steps/s"))
  , maxStepTiming(F("stepper step timing"), F("tics"), 1000000UL)
{
  plan_init();  // Initialize planner
  st_init();    // Initialize stepper
}

void Stepper::stop() {
  m_stopped = true;
  quickStop();
}

void Stepper::resume() {
  m_stopped = false;
}

int Stepper::add(float x, float y, float z, float e, float f) {

  // Check if we are stopped
  if (m_stopped) {
    logError << F("Unable to add movement to stepper, stepper has been stopped") << endl;
    return -1;
  }

  plan_buffer_line(x, y, z, e, f / 60);

  // Check again
  // Note: the stepper may have been stopped from an interrupt handler
  // (which is the case for tool detached), while normal execution was in
  // plan_buffer_line(). So, we check again, and remove the point we just
  // added, if necessary.
  if (m_stopped) {
    quickStop();
    logError << F("Unable to add movement to stepper, stepper was stopped") << endl;
    return -1;
  }

  // Success
  return 0;
}

void Stepper::outputStatus() {
  maxStepsComplete.outputStatus();
  maxInterruptsAllowed.outputStatus();
  maxCompletedWithoutTriggers.outputStatus();
  maxCompletedWithoutTriggersTics.outputStatus();
  maxStepRate.outputStatus();
  maxStepTiming.outputStatus();
}

void Stepper::periodicReport() {
  maxStepsComplete.reportIfChanged();
  maxInterruptsAllowed.reportIfChanged();
  maxCompletedWithoutTriggers.reportIfChanged();
  maxCompletedWithoutTriggersTics.reportIfChanged();
  maxStepRate.reportIfChanged();
  maxStepTiming.reportIfChanged();
}
