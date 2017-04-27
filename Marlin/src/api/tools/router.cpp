#include "../../../Marlin.h"
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
  SET_INPUT(ROUTER_COMMS_PIN);
  s_router_read.listen();
  s_router_read.begin(300);
}

static void writeMode() {
  s_router_read.end();
  SET_OUTPUT(ROUTER_COMMS_PIN);
  s_router_write.listen();
  s_router_write.begin(300);
}

static void ensureInitialized() {
  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    readMode();
  }
}

static bool confirmResponse(char* message){
  int try_count = 0;
  static const int timeout = 300; //ms
  readMode();
  static int wait_time = millis();
  while (try_count < 2) {

    if (wait_time-millis() > timeout){
      try_count++;
      writeMode();
      s_router_write.print(message);
      readMode();
      wait_time = millis();
    }

    while (s_router_read.available()>1) {
      if(s_router_read.read() != 1) {
        try_count++;
        writeMode();
        s_router_write.print(message);
        readMode();
        wait_time = millis();
        break;
      }
      return 1;
    }
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

static void s_sendRouterRotationSpeed(int percent) {
  SERIAL_ECHO_START;
  SERIAL_ECHO("Set rotation speed percentage to "); SERIAL_ECHOLN(percent);

  const int crc = CRC8(percent);
  ensureInitialized();


  char message[11];
  sprintf(message, "R%u %u\r\n", percent, crc);
  writeMode();
  s_router_write.print(message);
  if(confirmResponse(message)) {
    // DO WHAT YOU GOTTA DO
  }
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
    SERIAL_ECHOPGM("units\n");
  }

  if (tool != TOOLS_ROUTER) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set router rotation speed, tool is "); SERIAL_ERRORLN(toolTypeAsString(tool));
    return -1;
  }

  if ( speed > 100 || speed < 0) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set rotation speed to "); SERIAL_ERROR(speed);
    SERIAL_ERRORPGM("units, value is outside expected range\n");
    return -1;
  }

  s_rotationSpeed = speed;
  s_sendRouterRotationSpeed(s_rotationSpeed); // Need to set = speed first?

  return 0;
}
