echo Compiling V-One Firmware

#!/bin/bash

# Firmware version
VERSION=$(sed -n 's/^[[:space:]]*#define[[:space:]]*VERSION_STRING[[:space:]]*"\([^"]*\)"/\1/p' version.h)

# Define model variants
declare -A MODELS
declare -A VARIANT_SUFFIXES

MODELS["VONE_batch_0_TO_5"]=1
MODELS["VONE_batch_6_to_present"]=6
MODELS["EXPERIMENTAL"]=9999

VARIANT_SUFFIXES["VONE_batch_0_TO_5"]="_batch0to5"
VARIANT_SUFFIXES["VONE_batch_6_to_present"]="_batch6"
VARIANT_SUFFIXES["EXPERIMENTAL"]="_experimental"


# Build firmware for each model
for MODEL_NAME in "${!MODELS[@]}"; do
    MODEL_VALUE=${MODELS[$MODEL_NAME]}

    FIRMWARE_VARIANT_SUFFIX=${VARIANT_SUFFIXES[$MODEL_NAME]}

    echo "Compiling MODEL($MODEL_NAME -- $MODEL_VALUE) with FIRMWARE_VARIANT_SUFFIX($FIRMWARE_VARIANT_SUFFIX)"

    # Compile with arduino-cli
    arduino-cli compile -b arduino:avr:mega ./Marlin.ino \
        --build-property "build.extra_flags=-DMODEL=$MODEL_VALUE -DFIRMWARE_VARIANT_SUFFIX=\"$FIRMWARE_VARIANT_SUFFIX\"" \
        --output-dir ../build 

    # Rename output file
    mv ../build/Marlin.ino.hex "../build/${VERSION}${FIRMWARE_VARIANT_SUFFIX}.hex"

    echo "Built: ${VERSION}${FIRMWARE_VARIANT_SUFFIX}.hex"
done