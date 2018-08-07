#include "Stepper.h"

#include "../pins/endstops/Endstop.h"
#include "../pins/endstops/EndstopMonitor.h"

#include "../../../planner.h"
#include "../../../stepper.h"
#include "../../../serial.h"

Stepper::Stepper(
  EndstopMonitor& endstopMonitor
) : m_endstopMonitor(endstopMonitor) {
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

bool Stepper::isEndstopTrippered(const Endstop& endstop) const {
  return m_endstopMonitor.isEndstopTriggered(endstop);
}

void Stepper::resetEndstop(const Endstop& endstop) {
  m_endstopMonitor.resetEndstop(endstop);
}
