#pragma once
#include "Tool.h"

namespace tools {

class Dispenser : public Tool {
    public:
        virtual const char* typeName() override { return "Dispenser"; } 
        
        virtual int prepare() override;
        virtual int unprepare() override;

        int setDispenseHeight(float height);
        float dispenseHeight();

        // Additional method, used to apply dispense height
        int asyncMove(
            float x, float y, float z, 
            float e, float f, 
            bool applyDispenseHeight = false
        );

    private: 
        float m_dispenseHeight = 0.0f;
};

}
