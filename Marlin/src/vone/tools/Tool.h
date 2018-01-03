#pragma once

class Stepper;
namespace tools {

class Tool {
  public:
    Tool(Stepper& stepper);

    virtual const char* name() const = 0;

    void attach();
    void detach();
    bool attached() const { return m_attached; }
    bool detached() const { return !m_attached; }

    int prepareToMove();
    int resetPreparations();
    bool prepared() const { return m_prepared; }

    virtual int enqueueMove(float x, float y, float z, float e, float f) = 0;

  protected:
    Stepper& m_stepper;

  private:
    bool m_prepared = false;
    volatile bool m_attached = false;

    virtual int prepareToMoveImpl() = 0;
    virtual int resetPreparationsImpl() = 0;
};

}
