#pragma once

#include "pins/PTopPin/PTopPin.h"

class VOne {
  public:
    VOne(
      int ptopPin
    );

  private:
    struct Pins {
      PTopPin ptop;

      Pins(
        int ptopPin
      );
    } pins;

};
