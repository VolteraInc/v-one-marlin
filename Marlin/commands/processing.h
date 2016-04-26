#pragma once

#include "../Configuration_adv.h"

// TODO: enhance this interface so that we don't need to expose all these vars
extern char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
extern char* strchr_pointer;
extern int bufindr;

bool command_prefix_seen(char prefix); // check if the given prefix matches the first letter of the current command
bool code_seen(char code); // find the given code
float code_value(); // the float that follows the found code
long code_value_long(); // the long that follows the found code
char code_prefix(); // the character before the found code

//int process_gcode();
//int process_mcode();
int process_vcode(int command_code);
