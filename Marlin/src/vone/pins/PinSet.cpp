#include "PinSet.h"

#include "../../../serial.h"
#include "../../../Marlin.h"

static void s_outputPin(const char* name, bool value) {
  protocol 
    << name
    << (value ? F("TRIGGERED") : F("open"))
    << endl;
}

PinSet::PinSet(
  int ptopDigialPin,
  int ptopAnalogPin,
  int bedTemperatureAnalogPin,
  int heaterDigitalPin
)
  : ptop(ptopDigialPin, ptopAnalogPin)
  , bedTemperature(bedTemperatureAnalogPin)
  , heater(heaterDigitalPin)
{
}

int PinSet::outputEndStopStatus() {
  bool ptopValue = false;
  int returnValue = ptop.readDigitalValue(ptopValue);

  // Note: this output is used in manufacturing scripts
  protocol << F("Reporting endstop status") << endl;

  s_outputPin("x_min: ", READ_PIN(X_MIN));
  s_outputPin("y_min: ", READ_PIN(Y_MIN));

  s_outputPin("z_min: ", READ_PIN(Z_MIN));
  s_outputPin("z_max: ", READ_PIN(Z_MAX));

  s_outputPin("p_top: ", ptopValue);
  s_outputPin("p_bot: ", READ_PIN(P_BOT));

  s_outputPin("xy_min_x: ", READ_PIN(XY_MIN_X));
  s_outputPin("xy_max_x: ", READ_PIN(XY_MAX_X));
  s_outputPin("xy_min_y: ", READ_PIN(XY_MIN_Y));
  s_outputPin("xy_max_y: ", READ_PIN(XY_MAX_Y));

  return returnValue;
}
