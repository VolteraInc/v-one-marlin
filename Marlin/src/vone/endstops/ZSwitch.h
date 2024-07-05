#pragma once

#include "Endstop.h"

class ZSwitch : public Endstop {

  public:
    enum class Type {
      UseDefault = -1,

      // The original z-switch had a weaker spring
      // than the probe, so it would trigger before
      // the probe -- in 2019 data revealed that was
      // causing inconsistent heights when homing z
      Original = 0,

      // Used from batch 7 on, also
      // available as an hardware upgrade
      StrongerSpring = 1,

      // Used for batch 8 on, new control board and
      //XYZ using carbon strain gauges
      StrainGauge = 2

    };


  private:
    Type m_type;

  public:
    static inline const __FlashStringHelper* typeName(Type type) {
      using zt = ZSwitch::Type;

      switch (type) {
        case zt::Original:
          return F("original");

        case zt::StrongerSpring:
          return F("stronger spring");

        case zt::StrainGauge:
          return F("XYZ strain gauge");

        case zt::UseDefault:
        default:
          return F("use default");
      }
    }

    static ZSwitch::Type determineType(ZSwitch::Type type, long batchNumber) {
      using zt = ZSwitch::Type;

      if (type != zt::UseDefault) {
        return type;
      }

      if (batchNumber > 7 )
      {
        return zt::StrainGauge;
      } 
      else if (batchNumber > 6)
      {
        return zt::StrongerSpring;
      }
      else
      {
        return zt::Original;
      }
    }

    static ZSwitch::Type toType(int value) {
      using zt = ZSwitch::Type;

      switch(value) {
        case 0: return zt::Original;
        case 1: return zt::StrongerSpring;
        case 2: return zt::StrainGauge;
        default: return zt::UseDefault;
      }
    }


    ZSwitch(
      Type type,
      const __FlashStringHelper* name,
      int pin,
      AxisEnum axis,
      int direction,
      bool inverted,
      bool virtualEndstop
    )
      : Endstop(name, pin, axis, direction, inverted, virtualEndstop)
      , m_type(type)
    {}

    bool isSpringStrongerThanProbe() const {
      return m_type == Type::StrongerSpring;
    }

    const __FlashStringHelper*  typeName() const {
      return typeName(m_type);
    }
};
