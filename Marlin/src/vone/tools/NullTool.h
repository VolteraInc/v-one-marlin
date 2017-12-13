#pragma once
#include "Tool.h"

namespace tools {

class NullTool : public Tool {
    public:
        virtual const char* typeName() override { return "None"; } 
        virtual int attached() override;
        virtual int prepare() override;
        virtual int unprepare() override;
};

}
