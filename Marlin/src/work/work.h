

extern unsigned long previous_millis_serial_rx;
extern unsigned long stepper_inactive_time;
void refresh_cmd_timeout();
void refresh_serial_rx_timeout();
void manage_inactivity();

extern bool pending_temp_change;
extern bool glow_led_override;
void leds_init();
void glow_leds();

void periodic_output();

void reportBufferEmpty();

void checkForEndstopHits();

void process_serial_commands();
