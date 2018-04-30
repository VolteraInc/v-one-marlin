#include "./CommandQueue.h"

#include "../../Marlin.h"

void CommandQueue::push(const char* command) {
  if (full()) {
    logError << F("Unable to process command, command queue is full -- ") << command << endl;
    return;
  }

  strcpy(&commands[write_index][0], command);
  ++commands_in_queue;
  ++write_index;
  if (write_index == BUFSIZE) {
    write_index = 0;
  }

  if (logging_enabled) {
    log
      << F("Enqueued command '") << command
      << F("' commands_in_queue=") << commands_in_queue
      << F(" write_index=") << write_index
      << endl;
  }
}

void CommandQueue::pop() {
  if (logging_enabled) {
    log << F("Dequeued command '") << front() << F("'") << endl;
  }

  commands[read_index][0] = 0; // write a empty string to make it clear this command is done
  --commands_in_queue;
  ++read_index;
  if (read_index == BUFSIZE) {
    read_index = 0;
  }

  if (logging_enabled) {
    log
      << F(" commands_in_queue=") << commands_in_queue
      << F(" read_index=") << read_index
      << endl;
  }
}

void CommandQueue::flush() {
  while (!empty()) {
    pop();
  }
}
