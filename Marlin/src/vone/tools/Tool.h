#pragma once

class Stepper;
namespace tools {

namespace PrepareToMove {
  enum class Option {
    startOnly,
    eOnly,
    skipCalibrateXYZ,
    none
  };
}

class Tool {
  public:
    Tool(Stepper& stepper);

    virtual const char* name() const = 0;

    void attach();
    void detach();
    bool attached() const { return m_attached; }
    bool detached() const { return !m_attached; }

    int prepareToMove(PrepareToMove::Option option = PrepareToMove::Option::none);
    int resetPreparations();
    bool prepared() const { return m_prepare_Completed; }

    virtual int enqueueMove(float x, float y, float z, float e, float f) = 0;

  protected:
    Stepper& m_stepper;

  private:
    bool m_prepare_Started = false;
    bool m_prepare_HomedXY = false;
    bool m_prepare_Completed = false;

    volatile bool m_attached = false;

    virtual int prepareToMoveImpl_Start() = 0;
    virtual int prepareToMoveImpl_HomeXY() = 0;
    virtual int prepareToMoveImpl_CalibrateXYZ() = 0;
    virtual int resetPreparationsImpl() = 0;
};

}
