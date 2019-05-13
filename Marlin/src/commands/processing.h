#pragma once

#include "CommandQueue.h"

extern CommandQueue command_queue;

float parseFloatArg(char argCode, float defaultValue);
long parseLongArg(char argCode, long defaultValue);

// The string value that follows the given code
// value ends at space or end of string
const char* parseStringArg(char code, char value[], int maxLen, const char* defaultValue);


bool command_prefix_seen(char prefix); // check if the given prefix matches the first letter of the current command
bool code_seen(char code); // find the given code

float code_value(); // the float that follows the found code
long code_value_long(); // the long that follows the found code
const char* code_value_raw(); // pointer to the char that follows the found code

char code_prefix(); // the character before the found code


int process_gcode(int command_code = -1);
int process_mcode(int command_code = -1);
int process_vcode(int command_code = -1);
int process_dcode(int command_code = -1);
int process_icode(int command_code = -1);
