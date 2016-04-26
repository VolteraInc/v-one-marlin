#include "processing.h"

#include <stdlib.h>
#include <string.h>

char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
char *strchr_pointer = nullptr; // a pointer to find chars in the command string like X, Y, Z, E, etc
int bufindr = 0;

bool command_prefix_seen(char prefix){
  strchr_pointer = &cmdbuffer[bufindr][0];
  return strchr_pointer[0] == prefix;
}

bool code_seen(char code) {
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != nullptr);  //Return true if the character was found
}

float code_value() {
  return strtod(strchr_pointer + 1, nullptr);
}

long code_value_long() {
  return strtol(strchr_pointer + 1, nullptr, 10);
}

char code_prefix() {
  if (!strchr_pointer) {
    return '\0';
  }

  const int index = strchr_pointer - 1 - &cmdbuffer[bufindr][0];
  if (index < 0 || index >= MAX_CMD_SIZE ) {
    return '\0';
  }

  return cmdbuffer[bufindr][index];
}
