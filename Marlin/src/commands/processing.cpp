#include "processing.h"

#include <stdlib.h>
#include <string.h>

#include "Arduino.h" // millis()

CommandQueue command_queue;
static const char* code_pointer = nullptr; // a pointer to find chars in the command string like X, Y, Z, E, etc

bool command_prefix_seen(char prefix){
  const char* ptr = command_queue.front();

  // skip spaces
  while (*ptr == ' ') {
    ++ptr;
  }

  if (*ptr == prefix) {
    code_pointer = ptr;
    return true;
  } else {
    code_pointer = nullptr;
    return false;
  }
}

bool code_seen(char code) {
  code_pointer = strchr(command_queue.front(), code);
  return (code_pointer != nullptr);  //Return true if the character was found
}

float code_value() {
  return code_pointer ? strtod(code_pointer + 1, nullptr) : 0.0f;
}

long code_value_long() {
  return code_pointer ? strtol(code_pointer + 1, nullptr, 10) : 0l;
}

const char* code_value_raw() {
  return code_pointer ? code_pointer + 1 : nullptr;
}

char code_prefix() {
  if (!code_pointer) {
    return '\0';
  }

  const char* command = command_queue.front();
  const int index = code_pointer - command - 1 ;
  if (index < 0 || index >= MAX_CMD_SIZE ) {
    return '\0';
  }

  return command[index];
}


unsigned long previous_millis_active_cmd = 0;
void refresh_cmd_timeout() {
  previous_millis_active_cmd = millis();
}
