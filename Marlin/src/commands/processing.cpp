#include "processing.h"

#include <stdlib.h>
#include <string.h>

#include "../../MarlinConfig.h" // millis()

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

float parseFloatArg(char argCode, float defaultValue) {
  const char* ptr = strchr(command_queue.front(), argCode);
  if (ptr == nullptr) {
    return defaultValue;
  }

  return strtod(ptr + 1, nullptr);
}

long parseLongArg(char argCode, long defaultValue) {
  const char* ptr = strchr(command_queue.front(), argCode);
  if (ptr == nullptr) {
    return defaultValue;
  }

  return strtol(ptr + 1, nullptr, 10);
}

const char* parseStringArg(char argCode, char value[], int maxLen, const char* defaultValue) {
  const char* argPtr = strchr(command_queue.front(), argCode);
  if (argPtr == nullptr) {
    return defaultValue;
  }

  int idx = 0;
  auto ptr = argPtr + 1;
  while (*ptr != '\0' && *ptr != ' ' && idx < maxLen - 1) {
    value[idx++] = *ptr;
    ++ptr;
  }
  value[idx] = '\0';

  return value;
}
