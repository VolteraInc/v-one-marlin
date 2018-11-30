#include "VOne.h"

#include "../libraries/MemoryFree/MemoryFree.h"

VOne::VOne(
  int ptopDigialPin,
  int ptopAnalogPin,
  int bedTemperaturePin,
  int heaterDigitalPin
)
  : pins(ptopDigialPin, ptopAnalogPin, bedTemperaturePin, heaterDigitalPin)

  , adc(pins.ptop, pins.bedTemperature)
  , heater(pins.heater, pins.bedTemperature)

  , m_endstopMonitor(endstops)
  , stepper(m_endstopMonitor)

  , toolBox(stepper, pins.ptop, endstops.toolSwitch)
  , toolDetector(toolBox, pins.ptop)

  , m_memoryUsage(F("memory usage"), F(" bytes"), 8192)
{
  // Configure calling frequency of TIMER0_COMPB_vect
  // NOTE: Timer 0 is used by millis() so don't change the prescaler
  OCR0B = 128;
  ENABLE_TEMPERATURE_INTERRUPT();
}

void VOne::start() {
  // ISRs are enabled on boot so we can't do this in the ctor (cleanly)
  // Note: The concern is that enabling ISRs in the ctor would allow them
  //       to run before the vone object is initialized. Fixing the
  //       initializing sequence is much cleaner than adding conditions to
  //       the ISRs
  ENABLE_TEMPERATURE_INTERRUPT();
  stepper.start();
}

void VOne::updateStats() {
  const auto now = millis();
  if (now > m_nextStatsCheckAt) {
    m_nextStatsCheckAt = now + 1000;
    m_memoryUsage.updateIfLower(freeMemory());
  }
}

// Perform work that must happen frequently but can be
// interrupted (briefly) by time critical work like
// serial character reads and the stepper.
// Note: work done in the main loop can be delayed delayed by
//       10s, 30s or longer. It depends on how long it takes
//       to process the current command.
void VOne::frequentInterruptibleWork() {

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

  updateStats();
}

void VOne::outputStatus() {
  m_memoryUsage.outputStatus();
  endstops.outputStatus();
  stepper.outputStatus();
}

void VOne::periodicReport() {
  m_memoryUsage.reportIfChanged();
  vone->stepper.periodicReport();
}

// See stepper.cpp for TIMER1_COMPA_vect

ISR(TIMER0_COMPB_vect) {
  // Allow other interrupts
  DISABLE_TEMPERATURE_INTERRUPT();
  interrupts();

  logging::inISR = true;
  vone->frequentInterruptibleWork();
  logging::inISR = false;

  // Restore interrupt settings
  // Notes:
  //   1) Disable interrupts before re-enable the frequentInterruptibleWork interrupt
  //      otherwise that interrupt might trigger/run now
  //   2) Global interrupts will be enabled when we exit this function
  noInterrupts();
  ENABLE_TEMPERATURE_INTERRUPT();
}
