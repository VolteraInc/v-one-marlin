#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"
#include "SoftwareSerial.h"


const int Tx = A2;
const int Rx = A3;
static bool enabled = false;

SoftwareSerial mySerial(Rx, Tx);

void drill_enable(){
  SET_OUTPUT(DRILL_PIN);
  mySerial.begin(300);
  enabled = true;
}
void drill_disable() {
  SET_INPUT(DRILL_PIN); //Pin is used as an analog read.
  enabled = false;
}

void drill_set_speed(int pulsewidth){
  mySerial.write(pulsewidth);
}
