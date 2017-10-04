#pragma once

#include "../../../../Marlin.h"
#include "../../pins/HeaterPin.h"
#include "../../pins/BedTemperaturePin/BedTemperaturePin.h"

class Heater {
  public:
    inline Heater(HeaterPin& heatPin, BedTemperaturePin &tempPin);

    float currentTemperature() { ScopedInterruptDisable sid; return currentTemp; };
    float targetTemperature() { ScopedInterruptDisable sid; return targetTemp; };
    inline void setTargetTemperature(float target);

    bool isHeating() { ScopedInterruptDisable sid; return currentTemp < targetTemp; };
    bool isCooling() { return !isHeating(); };

    inline void periodicInterruptibleWork();

  private:
    HeaterPin& heaterPin;
    BedTemperaturePin& temperaturePin;

    volatile float currentTemp = 0.0f;
    volatile float targetTemp = 0.0f;

    inline void _updateTemperature();
    inline void _updateHeating();
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

void Heater::_updateTemperature() {
  // Get temperature
  const auto temperature = temperaturePin.value().temperature;

  // Update volatile member
  {
    ScopedInterruptDisable sid;
    currentTemp = temperaturePin.value().temperature;
  }

  // Shut off the heater if the temperature is out of range
  // Notes: A temp below the minimum suggests the thermometer is broken
  if ( temperature <= BED_MINTEMP || temperature >= BED_MAXTEMP) {
    SERIAL_ERROR_START;
    SERIAL_PAIR("Unable to complete heating, current temperature is ", temperature);
    SERIAL_PAIR(" degrees, which is outside of the expected range, ", BED_MINTEMP);
    SERIAL_PAIR(" to ", BED_MAXTEMP);
    SERIAL_EOL;
    heaterPin.stopHeating();
  }
}

void Heater::_updateHeating() {
  // Turn heater on/off
  ScopedInterruptDisable sid;
  if (currentTemp < targetTemp) {
    heaterPin.startHeating();
  } else {
    heaterPin.stopHeating();
  }
}

void Heater::periodicInterruptibleWork() {
  // Run every 0.5s starting after 3s
  // Note: We wait 5s so we have time to collect a valid sample
  //       otherwise we'd need to check endTime != 0
  static unsigned long nextCheckAt = 5000;
  const auto now = millis();
  if (now < nextCheckAt) {
    return;
  }
  nextCheckAt = now + 500;

  _updateTemperature();
  _updateHeating();
}
