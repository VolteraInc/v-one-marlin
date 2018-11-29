#pragma once

#include <inttypes.h>
#include "../../../Axis.h"

#ifdef TRINAMIC_DRIVERS

#define XY_STEALTH_MAX_SPEED (220) // 300
#define XY_COOLSTEP_MIN_SPEED (186)
#define E_STEALTH_MAX_SPEED (400)
#define E_COOLSTEP_MIN_SPEED (160)

void trinamicInit();

void trinamicSetCurrent(AxisEnum axis, int current);
void trinamicSetSG(AxisEnum axis, int value);
void trinamicSetStealthMaxSpeed(AxisEnum axis, int max_speed);
void trinamicSetCoolstepMinSpeed(AxisEnum axis, int min_speed); // Not USED.

int trinamicGetSG(AxisEnum axis);
int trinamicGetCurrentScaling(AxisEnum axis);
uint32_t trinamicGetTStep(AxisEnum axis);


int trinamicGetStalled(AxisEnum axis);
int trinamicGetStallGuard(AxisEnum axis);
uint32_t trinamicGetDRVSTATUS(AxisEnum axis);
uint8_t trinamicGetGStat(AxisEnum axis);

#endif // TRINAMIC_DRIVERS
