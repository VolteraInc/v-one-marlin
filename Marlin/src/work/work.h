#pragma once

extern unsigned long previous_millis_serial_rx;
extern unsigned long stepper_inactive_time;
void refresh_cmd_timeout();
void refresh_serial_rx_timeout();
void manage_inactivity();

extern bool pending_temp_change;
void glow_leds();

void toolChanges();

void periodic_output();

void reportBufferEmpty();

void checkForEndstopHits();

void processSerialCommands();

void manufacturing_init();
void manufacturing_procedures();
