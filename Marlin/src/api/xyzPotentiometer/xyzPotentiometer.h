#pragma once

#include <Arduino.h>
#include "../../libraries/MCP43XX/MCP43XX.h"
#include "../../../pins.h"

#define X_POT_CHANNEL 2
#define Y_POT_CHANNEL 1
#define Z_POT_CHANNEL 3

//use these to flip polarity of potentiometers if they're wired in reverse
#define X_POLARITY 1
#define Y_POLARITY 1
#define Z_POLARITY 1

#define XYZ_POT_RESISTANCE 100000

#define SAMPLE_DELAY_MS 10
#define NUM_SAMPLES 10
#define MAX_TUNE_ATTEMPTS 50

#define TARGET_VOLTAGE 5
#define VOLTAGE_TOLERANCE 0.5

