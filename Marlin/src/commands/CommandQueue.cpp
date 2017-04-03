#include "./CommandQueue.h"

#include "../../Marlin.h"

void CommandQueue::push(const char* command) {
  if (full()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to process command, command queue is full -- ");
    SERIAL_ERRORLN(command);
    return;
  }

  strcpy(&commands[write_index][0], command);
  ++write_index;
  if (write_index == BUFSIZE) {
    write_index = 0;
  }

  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Enqueued command '"); SERIAL_ECHO(command);
    SERIAL_ECHOPGM("' commands_in_queue="); SERIAL_ECHO(commands_in_queue);
    SERIAL_ECHOPGM(" write_index="); SERIAL_ECHOLN(write_index);
  }
}

void CommandQueue::pop() {
  if (logging_enabled) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Dequeued command '"); SERIAL_ECHO(front());
  }

  commands[read_index][0] = 0; // write a empty string to make it clear this command is done
  commands_in_queue -= 1;
  ++read_index;
  if (read_index == BUFSIZE) {
    read_index = 0;
  }

  if (logging_enabled) {
    SERIAL_ECHOPGM("' commands_in_queue="); SERIAL_ECHO(commands_in_queue);
    SERIAL_ECHOPGM(" read_index="); SERIAL_ECHOLN(read_index);
  }
}
