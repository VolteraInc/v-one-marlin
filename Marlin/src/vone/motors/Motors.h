#pragma once

#include "../../../MarlinConfig.h"
#include "../../../Axis.h"
#include "../../../pins.h"

void setHomedState(AxisEnum axis, int value);

// NOTE: fastio's WRITE expects a macros as an arg.
//       that's why we use a macro here instead of
//       C++ templated. There is a template-based
//       fastio out there, but i have not taken
//       the time to find and integrate it
#define IMPLEMENT_MOTOR(axis)                      \
class axis##AxisMotor {                            \
  public:                                          \
    axis##AxisMotor()                              \
      : name(F(#axis "-axis"))                     \
    {                                              \
      SET_OUTPUT(axis##_ENABLE_PIN);               \
      SET_OUTPUT(axis##_DIR_PIN);                  \
      off();                                       \
    }                                              \
                                                   \
    bool enabled() const {                         \
      return m_enabled;                            \
    }                                              \
                                                   \
    void on() {                                    \
      WRITE(axis##_ENABLE_PIN, axis##_ENABLE_ON);  \
      m_enabled = true;                            \
    }                                              \
                                                   \
    void off() {                                   \
      WRITE(axis##_ENABLE_PIN, !axis##_ENABLE_ON); \
      m_enabled = false;                           \
      if (axis##_AXIS != E_AXIS) {                 \
        setHomedState(axis##_AXIS, 0);             \
      }                                            \
    }                                              \
                                                   \
    const __FlashStringHelper* name;               \
                                                   \
  private:                                         \
    bool m_enabled = false;                        \
}

IMPLEMENT_MOTOR(X);
IMPLEMENT_MOTOR(Y);
IMPLEMENT_MOTOR(Z);
IMPLEMENT_MOTOR(E);

class Motors {
  public:
    XAxisMotor xAxis;
    YAxisMotor yAxis;
    ZAxisMotor zAxis;
    EAxisMotor eAxis;

    void on() {
      xAxis.on();
      yAxis.on();
      zAxis.on();
      eAxis.on();
    }

    void off() {
      xAxis.off();
      yAxis.off();
      zAxis.off();
      eAxis.off();
    }

    bool anyEnabled() {
      return (
        xAxis.enabled() ||
        yAxis.enabled() ||
        zAxis.enabled() ||
        eAxis.enabled()
      );
    }

    void outputStatus() const;
    void reportChanges();

  private:
    struct {
      bool xAxisEnabled = false;
      bool yAxisEnabled = false;
      bool zAxisEnabled = false;
      bool eAxisEnabled = false;
    } m_reportedStatus;
};
