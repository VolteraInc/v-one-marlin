#pragma once

#include "../../api/movement/movement.h"

namespace tools {

class Tool {
  public:
    Tool(const char* typeName, Stepper& stepper);

    virtual const char* typeName() const = 0;

    void activate();
    void deactivate() volatile;
    bool active() const { return m_active; }

    virtual int prepare() = 0;
    virtual int unprepare() = 0;

  private:
    volatile bool m_detached = false;
};

}
