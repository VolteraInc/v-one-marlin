#include "Tool.h"

#include "../../../serial.h"
#include "../stepper/Stepper.h"

tools::Tool::Tool(Stepper& stepper)
  : m_stepper(stepper)
{
}

void tools::Tool::attach() {
  m_attached = true;
}

void tools::Tool::detach() {
  m_attached = false;
  resetPreparations();
  m_stepper.stop();
}

int tools::Tool::prepareToMove() {
  if (!m_prepared) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Preparing tool");
    m_stepper.resume();
    if (prepareToMoveImpl()) {
      return -1;
    }
    m_prepared = true;
  }
  return 0;
}

int tools::Tool::resetPreparations() {
  if (m_prepared) {
    m_prepared = false;

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Reset tool preparations");
    return resetPreparationsImpl();
  }
  return 0;
}
