#pragma once

#include "../../Configuration.h"

// TODO: remove CommandQueue, we only handle one command at a time
//       handling multiple would require a different approach to handling
//       async errors (i.e. requesting a resend of only the most recent
//       message will not be enough).
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
  void flush();

private:
  char commands[BUFSIZE][MAX_CMD_SIZE];
  unsigned int commands_in_queue = 0;
  unsigned int write_index = 0;
  unsigned int read_index = 0;
};
