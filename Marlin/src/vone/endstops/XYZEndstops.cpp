#pragma once

#include "XYZEndstops.h"
#include "../../../serial.h"
#include "../../utils/ScopedInterruptDisable.h"

//local vars
//uint16_t _gain, _filter, _dataRate;

XYZSensor::XYZSensor(const Endstops& endstops)
{
    xyzSensor.setDataRate(7200);
    tuneXYZEndstops(endstops);
}


void XYZSensor::tuneXYZEndstops(const Endstops& endstops)
{
    const auto& xyzFront = &endstops.xyPositionerForward;
    const auto& xyzBack = &endstops.xyPositionerBack;
    const auto& xyzLeft = &endstops.xyPositionerLeft;
    const auto& xyzRight = &endstops.xyPositionerRight;
    const auto& zMin = &endstops.zSwitch;

    //log << F("tune tune") << endl;

    tuneXYZEndstop(*zMin);
    tuneXYZEndstop(*xyzFront);
    tuneXYZEndstop(*xyzBack);
    tuneXYZEndstop(*xyzLeft);
    tuneXYZEndstop(*xyzRight);
}

void XYZSensor::tuneXYZEndstop(const Endstop& endstop)
{
    if(endstop.axis == X_AXIS)
    {
        setChannel(endstop);
        _tuneXValue = 0;
        for(int i = 0; i < NUM_SAMPLES; i++)
        {
            _tuneXValue = _tuneXValue + xyzSensor.getADCData()/10;
        }
        //log << _tuneXValue << endl;
    }
    else if (endstop.axis == Y_AXIS)
    {
        setChannel(endstop);
        _tuneYValue = 0;
        for(int i = 0; i < NUM_SAMPLES; i++)
        {
            _tuneYValue = _tuneYValue + xyzSensor.getADCData()/10;
        }
        //log << _tuneYValue << endl;
    }
    else //Z_axis
    {
        setChannel(endstop);
        _tuneZValue = 0;
        for(int i = 0; i < NUM_SAMPLES; i++)
        {
            _tuneZValue = _tuneZValue + xyzSensor.getADCData()/10;
        }
        //log << _tuneZValue << endl;
    }
}

uint8_t XYZSensor::isXYZTouch(const Endstop& endstop) //currently implemented to just see if something is triggered in this axis, reducing computation time
{
    uint32_t analogReading = 0;
    int32_t compReading = 0;
    setChannel(endstop);
    analogReading = xyzSensor.getADCData();
    //log << analogReading << endl;

    if(endstop.axis == X_AXIS)
    {
        compReading = analogReading - _tuneXValue;
        return (abs(compReading) > TRIGGER_THRESHOLD_X ) ? 1 : 0;
    }
    else if (endstop.axis == Y_AXIS)
    {
        compReading = analogReading - _tuneYValue;
        return (abs(compReading) > TRIGGER_THRESHOLD_Y ) ? 1 : 0;
    }
    else if (endstop.axis == Z_AXIS)//Z_axis
    {
        compReading = analogReading - _tuneZValue;
        return (abs(compReading) > TRIGGER_THRESHOLD_Z ) ? 1 : 0;
    }
    else
    {
        return 1; //if we cannot find anything, pass true as this will prevent damage in the machine, how to throw exception from here?
    }

}

void XYZSensor::setChannel(const Endstop& endstop)
{
    
    if(endstop.axis == X_AXIS)
    {   
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

}
