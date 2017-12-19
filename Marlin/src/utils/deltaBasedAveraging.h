#pragma once

#include <stddef.h>

float average(const float data[], size_t size);
float filteredAverage(const float data[], size_t size, float maxDelta, unsigned* o_numMatches);
