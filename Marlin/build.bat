@echo off
setlocal enabledelayedexpansion

echo Compiling V-One Firmware
echo .BAT script composed by chat GPT and is untested-- .SH script has been tested so use at your own risk and remove this warning if confident.



:: Extract firmware version
for /f "tokens=2 delims=\"" %%A in ('findstr /R "^#define[ ]*VERSION_STRING" version.h') do set VERSION=%%A

:: Define model variants
set MODELS[0]=VONE_batch_0_TO_5
set MODELS[1]=VONE_batch_6_to_present
set MODELS[2]=EXPERIMENTAL

set MODEL_VALUES[VONE_batch_0_TO_5]=1
set MODEL_VALUES[VONE_batch_6_to_present]=6
set MODEL_VALUES[EXPERIMENTAL]=9999

set VARIANT_SUFFIXES[VONE_batch_0_TO_5]=_batch0to5
set VARIANT_SUFFIXES[VONE_batch_6_to_present]=_batch6
set VARIANT_SUFFIXES[EXPERIMENTAL]=_experimental

:: Build firmware for each model
for %%M in (0 1 2) do (
    set MODEL_NAME=!MODELS[%%M]!
    set MODEL_VALUE=!MODEL_VALUES[!MODEL_NAME!]!
    set FIRMWARE_VARIANT_SUFFIX=!VARIANT_SUFFIXES[!MODEL_NAME!]!

    echo Compiling MODEL(!MODEL_NAME! -- !MODEL_VALUE!) with FIRMWARE_VARIANT_SUFFIX(!FIRMWARE_VARIANT_SUFFIX!)

    :: Compile with arduino-cli
    arduino-cli compile -b arduino:avr:mega ./Marlin.ino ^
        --build-property "build.extra_flags=-DMODEL=!MODEL_VALUE! -DFIRMWARE_VARIANT_SUFFIX=\"!FIRMWARE_VARIANT_SUFFIX!\"" ^
        --output-dir ../build 

    :: Rename output file
    move /Y ../build/Marlin.ino.hex "../build/!VERSION!!FIRMWARE_VARIANT_SUFFIX!.hex"

    echo Built: !VERSION!!FIRMWARE_VARIANT_SUFFIX!.hex
)

echo .BAT script composed by chat GPT and is untested-- .SH script has been tested so use at your own risk and remove this warning if confident.
endlocal