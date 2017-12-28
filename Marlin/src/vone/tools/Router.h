#pragma once
#include "Tool.h"

class PTopPin;

namespace tools {

class Router : public Tool {
    public:
        Router(PTopPin& pin);

        virtual const char* typeName() override { return "Router"; } 
        virtual int prepare() override;
        virtual int unprepare() override;

        int setRotationSpeed(unsigned speed);
        float rotationSpeed() { return m_rotationSpeed; }

    private:
        PTopPin& m_pin;
        int m_rotationSpeed = 0;
        
        int _sendAndRampTo(int percent);
        int _stopRotation();
        int _stopRotationIfMounted();
};

}