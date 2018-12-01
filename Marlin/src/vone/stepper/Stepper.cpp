#include "Stepper.h"

#include "../endstops/Endstop.h"
#include "../endstops/EndstopMonitor.h"

#include "../../../planner.h"
#include "../../../stepper.h"
#include "../../../serial.h"
#include "./digipots.h"

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

  // Initialize Digipot Motor Current
  digiPotInit();

  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11);
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0);
  TCCR1A &= ~(3<<COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
}

void Stepper::start() {
  st_start();
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
