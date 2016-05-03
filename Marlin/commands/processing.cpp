#include "processing.h"

#include <stdlib.h>
#include <string.h>

#include "Arduino.h" // millis()

char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
char *strchr_pointer = nullptr; // a pointer to find chars in the command string like X, Y, Z, E, etc
int bufindr = 0;

bool command_prefix_seen(char prefix){
  strchr_pointer = &cmdbuffer[bufindr][0];

  // skip spaces
  while (*strchr_pointer == ' ') {
    ++strchr_pointer;
  }

  return *strchr_pointer == prefix;
}

bool code_seen(char code) {
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != nullptr);  //Return true if the character was found
}

float code_value() {
  return strchr_pointer ? strtod(strchr_pointer + 1, nullptr) : 0.0f;
}

long code_value_long() {
  return strchr_pointer ? strtol(strchr_pointer + 1, nullptr, 10) : 0l;
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

unsigned long previous_millis_active_cmd = 0;
void refresh_cmd_timeout() {
  previous_millis_active_cmd = millis();
}
