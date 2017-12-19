#include "../../serial.h"
#include "../commands/processing.h"
#include "work.h"

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

inline void requestResend(
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
    requestResend(
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
    requestResend(
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
    requestResend(
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
  static uint16_t expectedLineNumber = 0;
  static char buffer[MAX_CMD_SIZE];
  static int index = 0;
  auto tooLong = false;

  while (!command_queue.full()) {
    char ch = MYSERIAL.read();

    // No characters available
    if (ch == -1) {
      break;

    // End of command
    } else if (ch == '\n' || ch == '\r') {
      // Handle long lines
      if (tooLong) {
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Finished receiving long command");
      } else {
        // Add to command queue
        buffer[index] = 0; // terminate string
        const char* command = parse(buffer, index, expectedLineNumber);
        if (command) {
          ++expectedLineNumber;
          command_queue.push(command);
        }
      }

      // Reset the write position for the next command
      index = 0;

    // Command too long, report
    // Note: Report long commands when they happen, as opposed to
    //       waiting for the end of the command (which may never come)
    } else if (index >= (MAX_CMD_SIZE - 1)) {
      if (!tooLong) {
        tooLong = true;
        buffer[index] = 0; // terminate string (so we can include it in the error)
        SERIAL_ERROR_START;
        SERIAL_ERRORPGM("Unable to process command, command is too long, will ignore until end of command found --");
        SERIAL_ERRORLN(buffer);
      }

    // Add character to command
    } else {
      buffer[index++] = ch;
    }
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

    // Send Acknowledgement
    SERIAL_PROTOCOLLNPGM("ok");

    // Refresh the timeout after processing so that the user/sw
    // has then entire timeout duration to issue another command
    refresh_serial_rx_timeout();
    refresh_cmd_timeout();
  }
}
