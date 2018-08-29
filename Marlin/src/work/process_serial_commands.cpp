#include "../../serial.h"
#include "../commands/processing.h"
#include "work.h"
#include "../utils/crc8.h"

static int s_bufferIndex = 0;

inline const char* skipWhitespace(const char* ptr) {
  while (*ptr == ' ') {
    ++ptr;
  }
  return ptr;
}

static void s_requestResend(
  unsigned long expectedLineNumber,
  const __FlashStringHelper* pgmReason,
  const char* msg
) {
  protocol
    << F("Resend lineNumber:") << expectedLineNumber
    << F(", reason:\"") << pgmReason
    << F("\", message:\"") << msg
    << F("\"")
    << endl;
}

void s_sendResponseOk() {
  protocol << F("ok") << endl;
}

inline const char* parse(
  const char* msg,
  unsigned int len,
  unsigned int expectedLineNumber
) {
  // skip empty lines
  if (len == 0) {
    // We output a message, because this is a symptom that something is wrong
    logNotice << F("Received a blank command, ignoring") << endl;
    return nullptr;
  }

  // Locate message validation characters
  const char* commandStart = skipWhitespace(msg);
  const char* star = strchr(commandStart, '*');

  // Detect 'raw input' mode
  // Note: useful for debugging and allows printer to accept g-code directly
  static const char lineNumberChar = 'N';
  if (*commandStart != lineNumberChar && !star) {
    static bool s_warned = false;
    if (!s_warned) {
      logWarning
        << F("Command is missing message validation fields, assuming raw input mode (i.e. bypassing message validation)")
        << endl;
      s_warned = true;
    }
    return commandStart;
  }

  const auto fnRequestResend = [expectedLineNumber, msg](const __FlashStringHelper* pgmReason) {
    s_requestResend(expectedLineNumber, pgmReason, msg);
  };

  // static auto fakeResend = false;
  // fakeResend = !fakeResend;
  // if (fakeResend) {
  //   fnRequestResend(F("Fake resend"));
  //   return nullptr;
  // }

  // Check for missing characters
  // Note:
  //    1) Missing characters is a more specific problem than a crc failure.
  //       So we check for that first. This also makes a crc failure more
  //       meaningful (e.g. it's something other than a missing character).
  //    2) We include the crc characters in the length, so that we can
  //       treat a missing crc character as a 'missing character'
  //       (not a just crc error)
  const char* comma = star ? strchr(star, ',') : nullptr;
  if (!comma) {
    fnRequestResend(F("Missing message length"));
    return nullptr;
  }
  const auto expectedLength = strtol(comma + 1, nullptr, 10);
  const auto actuallength = (comma - msg);
  if (expectedLength != actuallength) {
    // Log the expected vs actual lengths (for debugging)
    log
      << F("Missing characters detected, expected != actual, ")
      << expectedLength << " != " << actuallength << ", "
      << F("message=")
      << msg
      << endl;

    // Request resent
    fnRequestResend(F("Missing characters"));
    return nullptr;
  }

  // Validate the checksum
  if (!star) {
    fnRequestResend(F("Missing checksum"));
    return nullptr;
  }
  const auto msgChecksum = strtol(star + 1, nullptr, 16);
  const auto computedChecksum = crc8(commandStart, star - msg);
  if (msgChecksum != computedChecksum) {
    fnRequestResend(F("Bad checksum"));
    return nullptr;
  }

  // Check line number
  // Note: A crc error is more specific than an line number error, so we
  //       check crc before this point. As such, a line number error is
  //       much more likely to point to the client and printer falling
  //       out of sync.
  const char* newCommandStart = nullptr;
  const auto lineNumber = strtol(commandStart + 1, const_cast<char**>(&newCommandStart), 10);
  if (*commandStart != lineNumberChar) {
    fnRequestResend(F("Missing line number"));
    return nullptr;
  }
  if (lineNumber != expectedLineNumber) {
    fnRequestResend(F("Line number does not match expected value"));
    return nullptr;
  }

  // Command is valid, skip spaces and return
  return skipWhitespace(newCommandStart);
}

static uint16_t s_expectedLineNumber = 1;
static void read_commands() {
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
        log << F("Finished receiving long command") << endl;
        s_sendResponseOk();

        s_tooLong = false;
      } else {
        // Add to command queue
        s_buffer[s_bufferIndex] = 0; // terminate string
        const char* command = parse(s_buffer, s_bufferIndex, s_expectedLineNumber);
        if (command) {
          command_queue.push(command);
        }
      }

      // Increment
      // Note: we increment if we
      //       - request re-send
      //       - send an ok now
      //       - if we accept the command (and will send an ok later)
      // TODO: smells weird, should eliminate command queue
      ++s_expectedLineNumber;

      // Reset the write position for the next command
      s_bufferIndex = 0;


    // Command too long, report
    // Note: Report long commands when they happen, as opposed to
    //       waiting for the end of the command (which may never come)
    } else if (s_bufferIndex >= (MAX_CMD_SIZE - 1)) {
      if (!s_tooLong) {
        s_tooLong = true;
        s_buffer[s_bufferIndex] = 0; // terminate string (so we can include it in the error)
        logError
          << F("Unable to process command, command is too long, ")
          << F("will ignore until end of command found --")
          << s_buffer
          << endl;
      }

    // Add character to command
    } else {
      s_buffer[s_bufferIndex++] = ch;
    }
  }
}

void flushSerialCommands() {
  log << F("Flushing commands") << endl;

  // Clear queued commands
  command_queue.flush();

  // Clear the message buffer
  // Note: might have a partial message there
  s_bufferIndex = 0;

  // Wait then flush
  // Note: There is a race between the app and this attempt to flush, so give
  //       the app some extra time to send. This also allows data that is
  //       in transit to arrive and be flushed.
  delay(1000);
  const auto numAvailable = MYSERIAL.available();
  MYSERIAL.flush();
  log << F("Flushed ") << numAvailable << (" bytes ") << endl;

  if (numAvailable > 0) {
    s_requestResend(
      s_expectedLineNumber,
      F("Flushed serial buffer"),
      "<unknown>"
    );
    ++s_expectedLineNumber;
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
    logNotice
      << F("Unknown command: \"") << command_queue.front() << F("\"")
      << endl;
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
    s_sendResponseOk();

    // Refresh the timeout after processing so that the user/sw
    // has then entire timeout duration to issue another command
    refresh_serial_rx_timeout();
    refresh_cmd_timeout();
  }
}
