#pragma once

#include "../../../../Marlin.h"
#include "../../pins/HeaterPin.h"
#include "../../pins/BedTemperaturePin/BedTemperaturePin.h"

class Heater {
  public:
    inline Heater(HeaterPin& heatPin, BedTemperaturePin &tempPin);

    float currentTemperature() { return currentTemp; };
    float targetTemperature() { return targetTemp; };
    inline void setTargetTemperature(float target);

    bool isHeating() { ScopedInterruptDisable sid; return currentTemp < targetTemp; };
    bool isCooling() { return !isHeating(); };

    inline void frequentInterruptibleWork();

  private:
    HeaterPin& heaterPin;
    BedTemperaturePin& temperaturePin;

    volatile float currentTemp = 0.0f;
    volatile float targetTemp = 0.0f;

    inline void updateHeating(float temperature);
};

Heater::Heater(HeaterPin& heatPin, BedTemperaturePin &tempPin)
  : heaterPin(heatPin)
  , temperaturePin(tempPin) {
}

void Heater::setTargetTemperature(float target) {
  {
    ScopedInterruptDisable sid;
    targetTemp = target;
  }
  SERIAL_ECHO_START;
  SERIAL_PAIR("New target Temperature: ", target);
  SERIAL_EOL;
};

void Heater::updateHeating(float temperature) {
  // Shut off the heater if the temperature is out of range
  // Notes: A temp below the minimum suggests the thermometer is broken
  if (temperature < BED_MINTEMP || temperature > BED_MAXTEMP) {
    SERIAL_ERROR_START;
    SERIAL_PAIR("Unable to operate heater, current temperature is ", temperature);
    SERIAL_PAIR(" degrees, which is outside of the expected range, ", BED_MINTEMP);
    SERIAL_PAIR(" to ", BED_MAXTEMP);
    SERIAL_EOL;
    heaterPin.stopHeating();
    return;
  }

  // Turn heater on/off
  if (temperature < targetTemp) {
    heaterPin.startHeating();
  } else {
    heaterPin.stopHeating();
  }
}

void Heater::frequentInterruptibleWork() {
  // Run every 0.5s
  static unsigned long nextCheckAt = 0;
  const auto now = millis();
  if (now < nextCheckAt) {
    return;
  }
  nextCheckAt = now + 500;

  // Get temperature then update heater
  currentTemp = temperaturePin.value().temperature;
  updateHeating(currentTemp);
}
