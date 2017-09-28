#include "VOne.h"

VOne::Pins::Pins(
  int ptopDigialPin,
  int ptopAnalogPin,
  int bedTemperatureAnalogPin
)
  : ptop(ptopDigialPin, ptopAnalogPin)
  , bedTemperature(bedTemperatureAnalogPin)
  , adc(ptop, bedTemperature)
{
}

static void s_outputPin(const char* name, bool value) {
  SERIAL_PROTOCOL(name);
  if (value) {
    SERIAL_PROTOCOLLNPGM("TRIGGERED");
  } else {
    SERIAL_PROTOCOLLNPGM("open");
  }
}

int VOne::Pins::outputEndStopStatus() {

  bool ptopValue = false;
  int returnValue = ptop.readDigitalValue(ptopValue);

  // Note: this output is used in manufacturing scripts
  SERIAL_PROTOCOLLNPGM("Reporting endstop status");

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

VOne::VOne(
  int ptopDigialPin,
  int ptopAnalogPin,
  int bedTemperaturePin
)
  : pins(ptopDigialPin, ptopAnalogPin, bedTemperaturePin)
{
}
