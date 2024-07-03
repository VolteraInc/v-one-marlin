#pragma once

#include "XYZEndstops.h"
#include "../../../serial.h"
#include "../../utils/ScopedInterruptDisable.h"

//local vars
//uint16_t _gain, _filter, _dataRate;

XYZSensor::XYZSensor(const Endstops& endstops)
{
    const auto& xyzFront = &endstops.xyPositionerForward;
    const auto& xyzBack = &endstops.xyPositionerBack;
    const auto& xyzLeft = &endstops.xyPositionerLeft;
    const auto& xyzRight = &endstops.xyPositionerRight;

    xyzSensor.setDataRate(7200);

    tuneXYZEndstop(*xyzFront);
    tuneXYZEndstop(*xyzBack);
    tuneXYZEndstop(*xyzLeft);
    tuneXYZEndstop(*xyzRight);

    log << F("TUNED XYZ") << endl;
}

void XYZSensor::tuneXYZEndstop(const Endstop& endstop)
{
    if(endstop.axis == X_AXIS)
    {
        //_channel = X;
        //log << F("Start tUNEX") << endl;
        setChannel(endstop);
        _tuneXValue = 0;
        for(int i = 0; i < NUM_SAMPLES; i++)
        {
            _tuneXValue =+ xyzSensor.getADCData()/10;
        }
         //log << F("End tUNEX") << endl;
        //implement method for flipping polarity?
    }
    else if (endstop.axis == Y_AXIS)
    {
        //_channel = Y;
        setChannel(endstop);
        _tuneYValue = 0;
        for(int i = 0; i < NUM_SAMPLES; i++)
        {
            _tuneYValue =+ xyzSensor.getADCData()/10;
        }

        //implement method for flipping polarity?
    }
    else //Z_axis
    {
        //_channel = Z;
        setChannel(endstop);
        _tuneZValue = 0;
        for(int i = 0; i < NUM_SAMPLES; i++)
        {
            _tuneYValue =+ xyzSensor.getADCData()/10;
        }

        //implement method for flipping polarity?
    }
}

uint8_t XYZSensor::isXYZTouch(const Endstop& endstop) //currently implemented to just see if something is triggered in this axis, reducing computation time
{
    //ScopedInterruptDisable sid;
    uint32_t analogReading = 0;

    //log << xyzSensor.readRegister(0x02) << endl; //let's check to see if SPI works here...
    //log << 11 << endl;


    setChannel(endstop);
    analogReading = xyzSensor.getADCData();
    //analogReading = _tuneXValue;
    log << analogReading << endl;
    
    if(endstop.axis == X_AXIS)
    {
        //if(abs(analogReading - _tuneXValue) > TRIGGER_THRESHOLD ){log << analogReading << endl;}

        return (abs(analogReading - _tuneXValue) > TRIGGER_THRESHOLD ) ? 1 : 0;
    }
    else if (endstop.axis == Y_AXIS)
    {
        return (abs(analogReading - _tuneYValue) > TRIGGER_THRESHOLD ) ? 0 : 1;
    }
    else if (endstop.axis == Z_AXIS)//Z_axis
    {
        return (abs(analogReading - _tuneZValue) > TRIGGER_THRESHOLD ) ? 0 : 1;
    }
    else
    {
        return 1; //if we cannot find anything, pass true as this will prevent damage in the machine, how to throw exception from here?
    }

}

void XYZSensor::setChannel(const Endstop& endstop)
{
    xyzSensor.stop();
    
    if(endstop.axis == X_AXIS)
    {
        //log << F("MUXX") << endl;
        if(this->_invertX)
        {
            xyzSensor.setMux(X_MUX_N, X_MUX_P);
        }
        else
        {
            xyzSensor.setMux(X_MUX_P, X_MUX_N);
            

        }
    }
    else if (endstop.axis == Y_AXIS)
    {
        //log << F("MUXY") << endl;
        if(_invertY)
        {
            xyzSensor.setMux(Y_MUX_N, Y_MUX_P);
        }
        else
        {
            xyzSensor.setMux(Y_MUX_P, Y_MUX_N);
        }
    }
    else //Z_axis
    {
        if(_invertZ)
        {
            xyzSensor.setMux(Z_MUX_N, Z_MUX_P);
        }
        else
        {
            xyzSensor.setMux(Z_MUX_P, Z_MUX_N);
        }
    }

    xyzSensor.start();
}
