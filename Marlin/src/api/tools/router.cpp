#include "../../../Marlin.h"
#include "../../../temperature.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"
#include <SoftwareSerial.h>

// ----------------------------------------------
// Router - serial comms


// baudrate of 300 is based on the rise and fall times of the capacitor on ptop
static const int baud = 300;

static const int dummy_pin = A3;
static SoftwareSerial s_router_read(ROUTER_COMMS_PIN, dummy_pin);
static SoftwareSerial s_router_write(dummy_pin, ROUTER_COMMS_PIN);

//

static void readMode() {
  s_router_write.end();
  set_p_top_mode(P_TOP_COMMS_READ_MODE);
  s_router_read.listen();
  s_router_read.begin(300);
}

static void writeMode() {
  s_router_read.end();
  set_p_top_mode(P_TOP_COMMS_WRITE_MODE);
  s_router_write.listen();
  s_router_write.begin(300);
}

static void normalMode() {
  s_router_write.end();
  s_router_read.end();
  set_p_top_mode(P_TOP_NORMAL_MODE);
}

enum AckState {
  ACK_NONE,
  ACK_O,
  ACK_OK,
  ACK_OK_CR,
  ACK_OK_CR_LF
};
static enum AckState updateAckState(enum AckState state, char ch) {
  switch (state) {
    default:        return ch == 'O' ? ACK_O : ACK_NONE;
    case ACK_O:     return ch == 'K' ? ACK_OK : ACK_NONE;
    case ACK_OK:    return ch == '\r' ? ACK_OK_CR : ACK_NONE;
    case ACK_OK_CR: return ch == '\n' ? ACK_OK_CR_LF : ACK_NONE;
  }
}

static uint8_t CRC8(uint8_t data) {
  uint8_t crc = 0x00;
  uint8_t extract = data;
  for (uint8_t tempI = 8; tempI; tempI--) {
    uint8_t sum = (crc ^ extract) & 0x01;
    crc >>= 1;
    if (sum) {
        crc ^= 0x8C;
    }
    extract>>=1;
  }
  return crc;
}

static int s_write(char* msg) {
  int returnValue = 0;
  int attempt = 1;
  do {
    SERIAL_ECHO_START;
    SERIAL_ECHO(attempt == 1 ? "Writing " : "Resending "); SERIAL_ECHOLN(msg);
    writeMode();
    s_router_write.print(msg);

    // Receive/parse acknowledgement characters, timeout eventually
    const auto now = millis();
    static const unsigned int timeout = 300; //ms
    const auto tryUntil = now + timeout;
    enum AckState ackState = ACK_NONE;
    readMode();
    while (tryUntil <= now) {
      if (s_router_read.available() > 1) {
        char ch = s_router_read.read();
        ackState = updateAckState(ackState, ch);
        if (ackState == ACK_OK_CR_LF) {
          goto DONE;
        }
      }
    }
  } while(++attempt <= 2);
  returnValue = -1; // failed to sent

DONE:
  normalMode();
  return returnValue;
}

static int s_sendRouterRotationSpeed(int percent) {
  SERIAL_ECHO_START;
  SERIAL_ECHO("Set rotation speed percentage to "); SERIAL_ECHOLN(percent);
  char message[11];
  const int crc = CRC8(percent);
  sprintf(message, "R%u %u\r\n", percent, crc);
  return s_write(message);
}


// ----------------------------------------------
// Router - tool
static int s_rotationSpeed = 0;

int prepareRouter(Tool tool) {
  const char* context = "prepare probe";
  return (
    setRotationSpeed(tool, 0) ||
    raise() ||
    confirmMountedAndNotTriggered(context, tool, TOOLS_ROUTER) ||
    ensureHomedInXY() ||
    ensureHomedInZ(tool) ||
    centerTool(tool) ||
    raise()
  );
}

float getRotationSpeed(Tool tool) {
  if (tool != TOOLS_ROUTER) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Warning: rotation speed requested for "); SERIAL_ERROR(toolTypeAsString(tool));
    SERIAL_ECHOPGM(" returning "); SERIAL_ECHOLN(s_rotationSpeed);
  }
  return s_rotationSpeed;
}

int setRotationSpeed(Tool tool, int speed) {
  if (logging_enabled) {
    SERIAL_ECHOPGM("Setting rotation speed to "); SERIAL_ECHO(speed);
    SERIAL_ECHOLNPGM(" percent");
  }

  if (tool != TOOLS_ROUTER) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set router rotation speed, tool is "); SERIAL_ERRORLN(toolTypeAsString(tool));
    return -1;
  }

  if ( speed > 100 || speed < 0) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set rotation speed to "); SERIAL_ERROR(speed);
    SERIAL_ERRORLNPGM(" percent, value is outside expected range");
    return -1;
  }

  // Send the speed to the router
  if (s_sendRouterRotationSpeed(speed)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to set the router's speed, confirm router is attached and powered");

    // Attempt to stop the router (just in case)
    if (speed != 0 && s_sendRouterRotationSpeed(0)) {
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Unable to confirm that router's speed was set to 0");
    }
    return -1;
  }

  // success
  s_rotationSpeed = speed;
  return 0;
}
