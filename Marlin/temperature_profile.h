#pragma once

int profile_validate_input(const int temperature, const int duration);
bool profile_empty();
float profile_remaining_time();
void profile_add(const int temperature, const int duration);
void profile_reset();

void manage_heating_profile();
