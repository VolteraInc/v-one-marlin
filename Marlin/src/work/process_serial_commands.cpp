// #include "version.h"
// #include "Marlin.h"
// #include "planner.h"
// #include "temperature.h"
// #include "temperature_profile.h"
// #include "watchdog.h"
// #include "ConfigurationStore.h"
// #include "language.h"
//
#include "../../Marlin.h"
#include "../../language.h"
#include "../commands/processing.h"
#include "work.h"

static void read_commands() {
  static char command[MAX_CMD_SIZE];
  static int index = 0;

  while (MYSERIAL.available() > 0 && !command_queue.full()) {
    char ch = MYSERIAL.read();

    // End of command
    if (ch == '\n' || ch == '\r') {
      // Handle empty lines
      if(index == 0 || index >= (MAX_CMD_SIZE - 1)) {
        // We output a message, because this is a symptom that something is wrong
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Received a blank command, ignoring");
      } else {
        // Add to command queue
        command[index] = 0; // terminate string
        command_queue.push(command);
      }

      // Reset the write position for the next command
      index = 0;

    // Command too long
    } else if (index >= (MAX_CMD_SIZE - 1)) {
      command[index] = 0; // terminate string (so we can include it in the error)
      index = 0; // reset write index
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to process command, command is too long, will ignore until end of command found --");
      SERIAL_ERRORLN(command);

    // Add character to command
    } else {
      command[index++] = ch;
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
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(command_queue.front());
    SERIAL_ECHOLNPGM("\"");
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
    SERIAL_PROTOCOLLNPGM(MSG_OK);

    // Refresh the timeout after processing so that the user/sw
    // has then entire timeout duration to issue another command
    refresh_cmd_timeout();
  }
}
