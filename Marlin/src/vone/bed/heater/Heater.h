#pragma once

#include "../../../../Marlin.h"
#include "../../pins/HeaterPin.h"
#include "../../pins/BedTemperaturePin/BedTemperaturePin.h"

class Heater {
  public:
    inline Heater(HeaterPin& heatPin, BedTemperaturePin &tempPin);

    float currentTemperature() { return m_currentTemp; };
    float targetTemperature() { return m_targetTemp; };
    inline void setTargetTemperature(float target);

    bool isHeating() { ScopedInterruptDisable sid; return m_currentTemp < m_targetTemp; };
    bool isCooling() { return !isHeating(); };

    inline void frequentInterruptibleWork();

  private:
    HeaterPin& m_heaterPin;
    BedTemperaturePin& m_temperaturePin;
    unsigned long m_nextCheckAt = 0;

    volatile float m_currentTemp = 0.0f;
    volatile float m_targetTemp = 0.0f;

    inline void updateHeating(float temperature);
};

Heater::Heater(HeaterPin& heaterPin, BedTemperaturePin &temperaturePin)
  : m_heaterPin(heaterPin)
  , m_temperaturePin(temperaturePin) {
}

void Heater::setTargetTemperature(float target) {
  {
    ScopedInterruptDisable sid;
    m_targetTemp = target;
  }
  log << F("New target Temperature: ") << target << endl;
};

void Heater::updateHeating(float temperature) {
  // Shut off the heater if the temperature is out of range
  // Notes: A temp below the minimum suggests the thermometer is broken
  if (temperature < BED_MINTEMP || temperature > BED_MAXTEMP) {
    logError
      << F("Unable to operate heater, current temperature is ") << temperature
      << F(" degrees, which is outside of the expected range, ") << BED_MINTEMP
      << F(" to ") << BED_MAXTEMP
      << endl;
    m_heaterPin.stopHeating();
    return;
  }

  // Turn heater on/off
  if (temperature < m_targetTemp) {
    m_heaterPin.startHeating();
  } else {
    m_heaterPin.stopHeating();
  }
}

void Heater::frequentInterruptibleWork() {
  // Run every 0.5s
  const auto now = millis();
  if (now < m_nextCheckAt) {
    return;
  }
  m_nextCheckAt = now + 500;

  // Update currentTemp (i.e. volatile member)
  m_currentTemp = m_temperaturePin.value().temperature;

  // Process the latest temperature
  updateHeating(m_currentTemp);
}
