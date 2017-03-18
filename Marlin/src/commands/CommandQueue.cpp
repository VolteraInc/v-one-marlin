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
}

void CommandQueue::pop() {
  commands[read_index][0] = 0; // write a empty string to make it clear this command is done
  commands_in_queue -= 1;
  ++read_index;
  if (read_index == BUFSIZE) {
    read_index = 0;
  }
}
