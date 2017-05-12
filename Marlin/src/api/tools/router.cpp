#include "../../../Marlin.h"
#include "../../../temperature.h"
#include "../../../stepper.h"
#include "../api.h"
#include "internal.h"

// ----------------------------------------------
// Router - serial comms

// baudrate of 300 is based on the rise and fall times of the capacitor on ptop (600-800 us)
static const int baud = 300;
static const int dummy_pin = A3;
static SoftwareSerial s_router_write(dummy_pin, ROUTER_COMMS_PIN);

static void readMode() {
  s_router_write.end();
  set_p_top_mode(P_TOP_COMMS_READ_MODE);
}

static void writeMode() {
  set_p_top_mode(P_TOP_COMMS_WRITE_MODE);
  s_router_write.begin(baud);
  s_router_write.listen();
}

static void normalMode() {
  s_router_write.end();
  set_p_top_mode(P_TOP_NORMAL_MODE);
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
  int return_value = -1;
  int attempt = 1;
  static const unsigned int timeout = 300; //ms
  static const long sample_period = 10;

  do {
    SERIAL_ECHO_START;
    SERIAL_ECHO(attempt == 1 ? "Writing " : "Resending "); SERIAL_ECHOLN(msg);
    writeMode();
    s_router_write.print(msg);
    readMode();

    const auto now = millis();
    auto last_sample = now;
    const auto tryUntil = now + timeout;
    int sample_count = 0;

    while (millis() <= tryUntil) {
        if(millis() - last_sample >= sample_period) {
        sample_count += !digitalRead(ROUTER_COMMS_PIN);
        last_sample = millis();
      }
    }

    if (sample_count >= 3) {
      return_value = 0;
      SERIAL_ECHOPGM("Confirmed on attempt "); SERIAL_ECHO(attempt);
      SERIAL_ECHOPGM(" with sample count "); SERIAL_ECHOLN(sample_count);
      goto DONE;
    }
  } while(++attempt <= 2);

DONE:
  normalMode();
  return return_value;
}

static int s_sendRouterRotationSpeed(int percent) {
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Set rotation speed percentage to "); SERIAL_ECHOLN(percent);
  char message[11];
  const int crc = CRC8(percent);
  sprintf(message, "R%u %u\r\n", percent, crc);
  return s_write(message);
}


// ----------------------------------------------
// Router - tool
static float s_rotationSpeed = 0.0f;

int prepareRouter(Tool tool) {
  const char* context = "prepare router";
  return (
    setRotationSpeed(tool, 0.0f) ||
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

int setRotationSpeed(Tool tool, float speed) {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Setting rotation speed to "); SERIAL_ECHO(speed);
    SERIAL_ECHOLNPGM(" percent");
  }

  if (tool != TOOLS_ROUTER) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to set router rotation speed, tool is "); SERIAL_ERRORLN(toolTypeAsString(tool));
    return -1;
  }

  if (speed == 0.0f) {
    s_rotationSpeed = speed;
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("TODO: Stop the router");

  } else if ( speed >= 80.0f || speed <= 120.0f) {
    s_rotationSpeed = speed;
    SERIAL_ECHO_START;
    SERIAL_ECHO("TODO: Set rotation speed to "); SERIAL_ECHOLN(s_rotationSpeed);

  } else {
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
