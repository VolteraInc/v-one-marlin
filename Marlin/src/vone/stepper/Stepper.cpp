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
  , maxCompletedWithoutTriggersTics(F("stepper isr completed without triggering"), F(" tics"), 200)
  , maxStepRate(F("stepper step rate"), F("steps/s"))
  , maxStepTiming(F("stepper step timing"), F(" tics"), 1000000UL)
{
  plan_init();  // Initialize planner
  st_init();    // Initialize stepper
}

void Stepper::stop(const __FlashStringHelper* reason) {
  ScopedInterruptDisable sid;
  m_stopReason = reason;
  quickStop();
}

void Stepper::resume() {
  ScopedInterruptDisable sid;
  m_stopReason = nullptr;
}

const __FlashStringHelper* Stepper::stopReason() const {
  ScopedInterruptDisable sid;
  return m_stopReason;
}

bool Stepper::stopped() const {
  return stopReason() != nullptr;
}

int Stepper::add(float x, float y, float z, float e, float f) {

  // Check if we are stopped
  const auto rejectReason = stopReason();
  if (rejectReason) {
    logError
      << F("Unable to perform movement, ")
      << rejectReason
      << endl;
    return -1;
  }

  plan_buffer_line(x, y, z, e, f / 60);

  // Check again
  // Note: the stepper may have been stopped from an interrupt handler
  // (which is the case for tool detached), while normal execution was in
  // plan_buffer_line(). So, we check again, and remove the point we just
  // added, if necessary.
  const auto revertReason = stopReason();
  if (revertReason) {
    quickStop();
    logError
      << F("Unable to perform movement, ")
      << revertReason
      << endl;
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
