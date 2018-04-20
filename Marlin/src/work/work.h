#pragma once

// Inactivity
extern unsigned long previous_millis_serial_rx;
extern unsigned long stepper_inactive_time;
void refresh_cmd_timeout();
void refresh_serial_rx_timeout();
void manage_inactivity();

// LEDs
extern bool pending_temp_change;
void glow_leds();

// Manufacturing procedures
void manufacturing_init();
void manufacturing_procedures();

// Command processing
void processSerialCommands();
void checkForEndstopHits();
void flushAndRequestResend();

// Other
void periodic_output();
void reportBufferEmpty();
