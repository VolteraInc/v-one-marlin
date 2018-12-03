#pragma once

#include <inttypes.h>
#include "../../../Axis.h"

#define XY_STEALTH_MAX_SPEED (220) // 300
#define XY_COOLSTEP_MIN_SPEED (186)
#define E_STEALTH_MAX_SPEED (400)
#define E_COOLSTEP_MIN_SPEED (160)

void trinamicInit();

void trinamicSetCurrent(int axis, int current);
void trinamicSetSG(int axis, int value);
void trinamicSetStealthMaxSpeed(int axis, int max_speed);
void trinamicSetCoolstepMinSpeed(int axis, int min_speed); // Not USED.

int trinamicGetSG(int axis);
int trinamicGetCurrentScaling(int axis);
uint32_t trinamicGetTStep(int axis);

int trinamicGetStalled(int axis);
int trinamicGetStallGuard(int axis);
uint32_t trinamicGetDRVSTATUS(int axis);
uint8_t trinamicGetGStat(int axis);
