#include "xyzPotentiometer.h"

MCP43XX xyzPotentiometer;
enum potAxis {X, Y, Z};

bool startupXYZPotentiometer()
{
    xyzPotentiometer = MCP43XX(XYZ_POT_CS_PIN, XYZ_POT_RESISTANCE);

    xyzPotentiometer.Startup(X_POT_CHANNEL);
    xyzPotentiometer.Startup(Y_POT_CHANNEL);
    xyzPotentiometer.Startup(Z_POT_CHANNEL);

    xyzPotentiometer.WiperConnect(X_POT_CHANNEL);
    xyzPotentiometer.WiperConnect(Y_POT_CHANNEL);
    xyzPotentiometer.WiperConnect(Z_POT_CHANNEL);

}

bool tuneXYZPot(potAxis _axis)
{
    uint8_t potNum;
    uint8_t analogInPin;
    uint8_t polarity;
    uint16_t analogReading;
    float analogVoltage;

    switch(_axis)
    {
        case X:
            potNum = X_POT_CHANNEL;
            analogInPin = X_TRIG_ANALOG_PIN;
            polarity = X_POLARITY;
            break;
        case Y:
            potNum = Y_POT_CHANNEL;
            analogInPin = Y_TRIG_ANALOG_PIN;
            polarity = Y_POLARITY;
            break; 
        case Z:
            potNum = Z_POT_CHANNEL;
            analogInPin = Z_TRIG_ANALOG_PIN;
            polarity = Z_POLARITY;
            break;
    }
    for(uint8_t i = 0; i < MAX_TUNE_ATTEMPTS; i++)
    {
        analogVoltage = readAnalogVoltage(analogInPin);
        analogVoltage -= TARGET_VOLTAGE; //analogVoltage now holds how close or far we are from the target voltage
        analogVoltage *= polarity; //allows for correction of backwards wired potentiometer
        
        if (abs(analogVoltage) <= VOLTAGE_TOLERANCE) //we are in tolerance
        {
            return true;
        }
        else if(analogVoltage > 0) //voltage read is too high
        {
            xyzPotentiometer.WiperDecrement(potNum);
        }
        else //voltage read is too low
        {
            xyzPotentiometer.WiperIncrement(potNum);
        }
    }
    return false; //wasn't able to reach required voltage
}

bool isTuneXYZPot(potAxis _axis)
{
    uint8_t analogInPin;
    uint16_t analogReading;

    switch(_axis)
    {
        case X:
            analogInPin = X_TRIG_ANALOG_PIN;
            break;
        case Y:
            analogInPin = Y_TRIG_ANALOG_PIN;
            break; 
        case Z:
            analogInPin = Z_TRIG_ANALOG_PIN;
            break;
    }
    
    analogReading = readAnalogVoltage(analogInPin);

    if(abs(analogReading) <= VOLTAGE_TOLERANCE){
        return true;
    }
    else
    {
        return false;
    }
}

bool saveTuneXYZPot() //save current configuration into the potentiometer for future default/startup config
{
    xyzPotentiometer.SaveNVWiperPosition();
}

//util
float readAnalogVoltage(uint8_t _pin)
{
    uint16_t analogReading = 0;

    for(uint8_t i = 0; i < NUM_SAMPLES; i++)
    {
        analogReading += analogRead(_pin);
        delay(SAMPLE_DELAY_MS);
    }

    analogReading /= NUM_SAMPLES;
    return valToVolt(analogReading);
}

float valToVolt (uint16_t val){

}

