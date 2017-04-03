#pragma once

#include "../../Configuration.h"

class CommandQueue {
public:
  CommandQueue() {
    // initialize the first entry to an empty string
    commands[read_index][0] = 0;
  };

  void push(const char* command);
  void pop();

  const char* front() { return &commands[read_index][0]; }
  bool full() { return commands_in_queue == BUFSIZE; }
  bool empty() { return commands_in_queue == 0; }

private:
  char commands[BUFSIZE][MAX_CMD_SIZE];
  unsigned int commands_in_queue = 0;
  unsigned int write_index = 0;
  unsigned int read_index = 0;
};
