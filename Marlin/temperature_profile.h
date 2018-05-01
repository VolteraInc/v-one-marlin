#pragma once

bool profile_empty();
float profile_remaining_time();
int profile_add(const int temperature, const int duration);
void profile_reset();

void manage_heating_profile();
