#pragma once

// Functions for store, reading settings from EEPROM

void Config_ResetDefault();
void Config_PrintSettings();
void Config_StoreSettings();
void Config_RetrieveSettings();

void Config_RetrieveCalibration();
void Config_PrintCalibration();
void Config_StoreCalibration();
