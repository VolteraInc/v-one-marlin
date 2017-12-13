#include "Stepper.h"

#include "../../../planner.h"
#include "../../../stepper.h"

Stepper::Stepper() {
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

  if (m_stopped) {
    return -1;
  }

  plan_buffer_line(x, y, z, e, f / 60);

  if (m_stopped) {
    stop(); // remove the point we just added
    return -1;
  }

  // Success
  return 0;
}
