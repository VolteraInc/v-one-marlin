#include "../../serial.h"
#include "../commands/processing.h"
#include "work.h"

static int s_bufferIndex = 0;

inline byte checksum(const char* start, const char* end) {
  byte checksum = 0;
  while (start != end) {
    checksum ^= *(start++);
  }
  return checksum;
}

inline void skipWhitespace(const char** ptr) {
  while (**ptr == ' ') {
    ++(*ptr);
  }
}

void s_requestResend(
  unsigned long expectedLineNumber,
  const char* pgmReason,
  const char* msg
) {
  SERIAL_PAIR("Resend lineNumber:", expectedLineNumber);
  SERIAL_PROTOCOLPGM(", reason:\""); serialprintPGM(pgmReason);
  SERIAL_PAIR("\", message:\"", msg);
  SERIAL_PROTOCOLPGM("\"");
  SERIAL_EOL;
}

void s_sendResponseOk() {
  SERIAL_PROTOCOLLNPGM("ok");
}

inline const char* parse(
  const char* msg,
  unsigned int len,
  unsigned int expectedLineNumber
) {
  // skip empty lines
  if (len == 0) {
    // We output a message, because this is a symptom that something is wrong
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("NOTICE: Received a blank command, ignoring");
    return nullptr;
  }

  // skip any leading spaces
  const char* commandStart = msg;
  skipWhitespace(&commandStart);

  // Return if no line number included
  if (*commandStart != 'N') {
    return commandStart;
  }

  // Confirm checksum present
  const char* star = strchr(commandStart, '*');
  if (!star) {
    s_requestResend(
      expectedLineNumber,
      PSTR("Missing checksum"),
      msg
    );
    return nullptr;
  }

  // Validate the checksum
  const auto msgChecksum = strtol(star + 1, nullptr, 10);
  const auto computedChecksum = checksum(commandStart, star);
  if (msgChecksum != computedChecksum) {
    s_requestResend(
      expectedLineNumber,
      PSTR("Bad checksum"),
      msg
    );
    return nullptr;
  }

  // Check line number
  const char* newCommandStart = nullptr;
  const auto lineNumber = strtol(commandStart + 1, const_cast<char**>(&newCommandStart), 10);
  if (lineNumber != expectedLineNumber) {
    s_requestResend(
      expectedLineNumber,
      PSTR("Line number does not match expected value"),
      msg
    );
    return nullptr;
  }

  // Command is valid
  skipWhitespace(&newCommandStart);
  return newCommandStart;
}

static void read_commands() {
  static uint16_t s_expectedLineNumber = 0;
  static char s_buffer[MAX_CMD_SIZE];
  static auto s_tooLong = false;

  while (!command_queue.full()) {
    char ch = MYSERIAL.read();

    // No characters available
    if (ch == -1) {
      break;

    // End of command
    } else if (ch == '\n' || ch == '\r') {
      // Handle long lines
      if (s_tooLong) {
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Finished receiving long command");
        s_tooLong = false;
      } else {
        // Add to command queue
        s_buffer[s_bufferIndex] = 0; // terminate string
        const char* command = parse(s_buffer, s_bufferIndex, s_expectedLineNumber);
        if (command) {
          ++expectedLineNumber;
          command_queue.push(command);
        }
      }

      // Reset the write position for the next command
      s_bufferIndex = 0;


    // Command too long, report
    // Note: Report long commands when they happen, as opposed to
    //       waiting for the end of the command (which may never come)
    } else if (s_index >= (MAX_CMD_SIZE - 1)) {
      if (!s_tooLong) {
        s_tooLong = true;
        s_buffer[s_bufferIndex] = 0; // terminate string (so we can include it in the error)
        SERIAL_ERROR_START;
        SERIAL_ERRORPGM("Unable to process command, command is too long, will ignore until end of command found --");
        SERIAL_ERRORLN(s_buffer);
      }

    // Add character to command
    } else {
      s_buffer[s_bufferIndex++] = ch;
    }
  }
}

void flushSerialCommands() {
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Flushing commands");
  SERIAL_EOL;


  // Clear the message buffer
  // Note: might have a partial message there
  s_bufferIndex = 0;

  // Flush for a little while
  // Note:
  //   1) There is a race between the app and this attempt to flush, so give
  //      the app some extra time to send. This also allows data that is
  //      in transit to arrive and be flushed.
  //   2) We flush repeatedly just in case the app is sending a long message
  //   3) if new data arrives while flushing we'll need to request a resend
  auto flushUntil = millis() + 1000;
  for (auto cnt = 10; cnt--;) {
    shouldRequestResend |= MYSERIAL.available() != 0
    MYSERIAL.flush();
    delay(100);
  }

  // Request a resend, if needed
  if (shouldRequestResend) {
    requestResend(s_expectedLineNumber, PSTR("Flush and request resend"), "");
  } else {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Flush complete, no resend needed");
    SERIAL_EOL;
    return;
  }
}


static void process_command() {
  if        (command_prefix_seen('V')) { int code = (int)code_value(); process_vcode(code_seen('?') ? -1 : code);
  } else if (command_prefix_seen('D')) { int code = (int)code_value(); process_dcode(code_seen('?') ? -1 : code);
  } else if (command_prefix_seen('I')) { int code = (int)code_value(); process_icode(code_seen('?') ? -1 : code);
  } else if (command_prefix_seen('G')) { int code = (int)code_value(); process_gcode(code_seen('?') ? -1 : code);
  } else if (command_prefix_seen('M')) { int code = (int)code_value(); process_mcode(code_seen('?') ? -1 : code);
} else if (command_prefix_seen('?')) {
    // Output usage for common commands V and D
    // Notes:
    //   G- and M-commands are mostly for internal use
    //   I-Commands are intended for internal use (e.g. in production), misuse may harm the printer
    process_vcode();
    process_dcode();
  } else {
    SERIAL_ECHO_START;
    SERIAL_PAIR("Unknown command: \"", command_queue.front());
    SERIAL_ECHOPGM("\"");
    SERIAL_EOL;
  }
}

void processSerialCommands() {
  read_commands();

  if(!command_queue.empty()) {
    // Refresh the timeouts before processing so that that we have
    // the entire timeout duration to process the command
    // Note: this is necessary because some commands block
    // (aka busy-wait) but will still check for inactivity
    // (ie. they all call periodic_work())
    refresh_serial_rx_timeout();
    refresh_cmd_timeout();

    process_command();
    command_queue.pop();

    // Check end-stops so that synchronous commands
    // will report errors before sending OK
    checkForEndstopHits();

    // Send Acknowledgement
    SERIAL_PROTOCOLLNPGM("ok");

    // Refresh the timeout after processing so that the user/sw
    // has then entire timeout duration to issue another command
    refresh_serial_rx_timeout();
    refresh_cmd_timeout();
  }
}
