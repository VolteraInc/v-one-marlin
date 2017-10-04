#pragma once

#include <SoftwareSerial.h>

class CustomSerial {
public:
  CustomSerial(int pin);

  int send(char* msg, unsigned maxAttempts, unsigned* o_numAttempts = nullptr);

private:
  const int _pin;
  SoftwareSerial serial;
  long timeSent = 0;

  void _send(char* msg);
  void _recvAcknowmedgement(bool& acknowledged);
};
