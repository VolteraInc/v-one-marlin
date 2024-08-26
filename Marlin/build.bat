@echo off

@echo:
@echo Compiling V-One Firmware...
arduino-cli.exe compile -b arduino:avr:mega ./Marlin.ino --output-dir ../build

pause
