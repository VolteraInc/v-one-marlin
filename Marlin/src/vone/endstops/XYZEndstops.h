#pragma once

#include "../../libraries/ADS126X/ADS126X.h"
//#include "../VOne.h"
#include "Endstops.h"
#include "../../../pins.h"
#include "../../../Marlin.h"
#include "../../../stepper.h"

//hardware definitions
#define X_MUX_P 3
#define X_MUX_N 2
#define Y_MUX_P 5
#define Y_MUX_N 4
#define Z_MUX_P 7
#define Z_MUX_N 6

//when within this radius of expected XYZ, we start sensing the XYZ measurements
#define XYZ_PROX_X 10
#define XYZ_PROX_Y 10
#define XYZ_PROX_Z 5

#define TRIGGER_THRESHOLD_X 500
#define TRIGGER_THRESHOLD_Y 500
#define TRIGGER_THRESHOLD_Z 500
#define NUM_SAMPLES 10

class XYZSensor
{
    public:

        XYZSensor::XYZSensor(const Endstops& endstops);
        void tuneXYZEndstop(const Endstop& endstop);
        void tuneXYZEndstops(const Endstops& endstops);
        uint8_t isXYZTouch(const Endstop& endstop);
        void setChannel(const Endstop& endstop);

    private:
        uint32_t _tuneXValue, _tuneYValue, _tuneZValue;
        bool _invertX = true, _invertY = true, _invertZ = true;
        ADS126X xyzSensor = ADS126X(XYZ_CS_PIN, XYZ_START, ADC_PWDN, ADC_RST, XYZ_DATA_RDY);
        bool isNearXYZ();

};