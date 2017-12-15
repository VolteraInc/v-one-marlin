#pragma once

#include "PTopPin_main.h"

// ----------------------------------------------
// Communication

bool PTopPin::trytoSetMode_Communication() {
  ScopedInterruptDisable sid;
  if (mode == Mode::Idle) {
    mode = Mode::Communication;
    pinMode(_digitalPin, OUTPUT);
    return true;
  }
  preventAdcSampling = true;
  return false;
}

void PTopPin::_sendMessage(char* msg) {
  while (!trytoSetMode_Communication()) {
    delay(1);
  }

  SERIAL_ECHO_START;
  SERIAL_ECHO("Sending "); SERIAL_ECHOLN(msg);

  serial.begin(baud);
  serial.listen();
  serial.println(msg);
  timeSent = millis();
  serial.end();

  setMode_Idle();
}

int PTopPin::_recvAcknowledgement() {
  const unsigned long tryUntil = timeSent + 300; // ms
  auto count = 0u;
  auto ackCount = 0u;
  while (millis() <= tryUntil) {
    ++count;
    auto voltage = readValue().voltage;

    // Count low voltages (which signal an acknowledgement)
    // Note: actual value is 0.39
    if (voltage >= 0.29 && voltage <= 0.49) {
      ++ackCount;
    }
    delay(10);
  }

  // Note: if the tool resets it will be seen as an acknowledgement
  // this ensures that we don't retry, which could result in us
  // missing the reset. Handling this case more explicitly would
  // complicate this code for little gain.
  const bool acknowledged = ackCount > 3;

  SERIAL_ECHO_START;
  SERIAL_PAIR("Received ", acknowledged ? "ack" : "noAck");
  SERIAL_PAIR(", count = ", ackCount);
  SERIAL_PAIR(" of ", count);
  SERIAL_EOL;

  return acknowledged ? 0 : -1;
}

int PTopPin::send(char* msg) {
  const unsigned maxAttempts = 3;
  auto attempt = 1u;
  for (attempt = 1; attempt <= maxAttempts; ++attempt) {

    if (attempt > 1) {
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Retrying");
    }

    _sendMessage(msg);
    if (_recvAcknowledgement()) {
      continue;
    }

    // Log if we suceed close to the maximum number of retries
    // Note: seeing this message suggests that something unexpected is happening
    //       and/or the maximum number of retries is too low
    if (attempt > 2) {
      SERIAL_ECHO_START;
      SERIAL_PAIR("NOTICE: p-top message sent on attempt ", attempt);
      SERIAL_PAIR(" of ", maxAttempts);
      SERIAL_EOL;
    }

    // Success
    return 0;
  }
  return -1;
}
