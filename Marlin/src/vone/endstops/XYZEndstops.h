#pragma once

#include "../../libraries/ADS126X/ADS126X.h"
//#include "../VOne.h"
#include "Endstops.h"
#include "../../../pins.h"

//hardware definitions
#define X_MUX_P 3
#define X_MUX_N 2
#define Y_MUX_P 5
#define Y_MUX_N 4
#define Z_MUX_P 7
#define Z_MUX_N 6

#define TRIGGER_THRESHOLD 1000 //TUNE BACK DOWN
#define NUM_SAMPLES 10

class XYZSensor
{
    public:

        XYZSensor::XYZSensor(const Endstops& endstops);
        //void configXYZ(Endstops& endstops);
        void tuneXYZEndstop(const Endstop& endstop);
        uint8_t isXYZTouch(const Endstop& endstop);
        void setChannel(const Endstop& endstop);

    private:
        uint32_t _tuneXValue, _tuneYValue, _tuneZValue;
        bool _invertX = false, _invertY = false, _invertZ = false;
        ADS126X xyzSensor = ADS126X(XYZ_CS_PIN, XYZ_START, ADC_PWDN, ADC_RST, XYZ_DATA_RDY);

};