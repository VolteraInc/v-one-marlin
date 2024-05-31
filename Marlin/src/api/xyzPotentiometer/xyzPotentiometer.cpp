#include "xyzPotentiometer.h"

MCP43XX xyzPotentiometer = MCP43XX(XYZ_POT_CS_PIN, XYZ_POT_RESISTANCE);

bool startupXYZPotentiometer()
{
    xyzPotentiometer.Startup(X_POT_CHANNEL);
    xyzPotentiometer.Startup(Y_POT_CHANNEL);
    xyzPotentiometer.Startup(Z_POT_CHANNEL);

    xyzPotentiometer.WiperConnect(X_POT_CHANNEL);
    xyzPotentiometer.WiperConnect(Y_POT_CHANNEL);
    xyzPotentiometer.WiperConnect(Z_POT_CHANNEL);
}

bool tuneXYZPot(potAxis _axis)
{
    uint8_t potNum, potPos, analogInPin;
    int8_t polarity;
    float analogVoltage, analogDiff;

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

    //read current state of circuit
    analogVoltage = readAnalogVoltage(analogInPin);
    potPos = xyzPotentiometer.GetWiperPosition(potNum);
    
    do
    {
        //how far off tolerance are we currently
        analogDiff = analogVoltage - TARGET_VOLTAGE*VOLTAGE_DIVIDER;
        analogDiff *= polarity; //allows for correction of backwards wired potentiometer

        if(analogDiff > 0) //voltage read is too high
        {
            xyzPotentiometer.WiperDecrement(potNum);
            potPos--;
            //Serial.println(analogVoltage);
            
            //check if next sample got us closer to tolerance
            analogVoltage = readAnalogVoltage(analogInPin);
            if( abs(analogVoltage - TARGET_VOLTAGE*VOLTAGE_DIVIDER) > abs (analogDiff) ) //if true, we are getting farther out of tolerance
            {
                xyzPotentiometer.WiperIncrement(potNum); //go back a step to correct overcorrection
                break; //exit loop
            }
            
        }
        else //voltage read is too low
        {
            xyzPotentiometer.WiperIncrement(potNum);
            potPos++;
            //Serial.println(analogVoltage);

            //check if next sample got us closer to tolerance
            analogVoltage = readAnalogVoltage(analogInPin);
            if( abs(analogVoltage - TARGET_VOLTAGE*VOLTAGE_DIVIDER) > abs (analogDiff) ) //if true, we are getting farther out of tolerance
            {
                xyzPotentiometer.WiperDecrement(potNum); //go back a step to correct overcorrection
                break; //exit loop
            }
        }
    }
    while(potPos > MIN_POT_POS && potPos < MAX_POT_POS);

    //evaluate tuning
    return isTuneXYZPot(_axis);
} 

bool isTuneXYZPot(potAxis _axis)
{
    uint8_t analogInPin;
    uint16_t analogVoltage;

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
    
    analogVoltage = readAnalogVoltage(analogInPin);
    analogVoltage -= TARGET_VOLTAGE*VOLTAGE_DIVIDER;

    if(abs(analogVoltage) <= VOLTAGE_TOLERANCE*VOLTAGE_DIVIDER){
        return true;
    }
    else
    {
        return false;
    }
}
/*
bool saveTuneXYZPot() //CURRENTLY NOT WORKING
{
    xyzPotentiometer.SaveNVWiperPosition();
}*/

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
    return ADC_REFERENCE_VOLTAGE*(((float)val)/ADC_MAX_CODE);
}