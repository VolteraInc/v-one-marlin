#pragma once

#include "../../Configuration.h"

class CommandQueue {
public:
  void push(const char* command);
  void pop();

  const char* front() { return &commands[read_index][0]; }
  bool full() { return commands_in_queue < BUFSIZE; }
  bool empty() { return commands_in_queue > 0; }

private:
  char commands[BUFSIZE][MAX_CMD_SIZE];
  unsigned int commands_in_queue;
  unsigned int write_index;
  unsigned int read_index;
};
