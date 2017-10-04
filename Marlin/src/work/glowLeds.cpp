#include "../../Configuration.h"
#include "../vone/Vone.h" // temperatures
#include "work.h" // previous_millis_serial_rx HACK

bool pending_temp_change = false;

static struct {
  char r,g,b;
  short pace;
} s_overrideLeds;

int overrideLeds(char r, char g, char b, short pace) {
  s_overrideLeds.r = r;
  s_overrideLeds.g = g;
  s_overrideLeds.b = b;
  s_overrideLeds.pace = pace;
  return 0;
}

static bool overridingLeds() {
  return s_overrideLeds.r || s_overrideLeds.b || s_overrideLeds.g;
}

void glow_leds() {
  #define GLOW_LED_COUNT 3
  #define TEMP_PACE_CURVE max((-5 * bedTemp) / 42 + 36, 5)
  static const char glow_led_pins[GLOW_LED_COUNT] = {LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN};
  static const unsigned char sin_lookup[256] = {43,45,48,51,53,56,58,61,64,66,69,72,74,77,79,82,85,87,90,92,95,97,100,102,105,107,110,112,115,117,120,122,125,127,130,132,134,137,139,141,144,146,148,151,153,155,157,159,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190,192,194,196,198,199,201,203,205,206,208,209,211,213,214,216,217,219,220,222,223,224,226,227,228,230,231,232,233,234,235,237,238,239,240,241,242,242,243,244,245,246,246,247,248,249,249,250,250,251,251,252,252,253,253,253,254,254,254,254,255,255,255,255,255,255,255,255,255,255,255,254,254,254,254,253,253,253,252,252,251,251,250,250,249,249,248,247,246,246,245,244,243,242,242,241,240,239,238,237,235,234,233,232,231,230,228,227,226,224,223,222,220,219,217,216,214,213,211,209,208,206,205,203,201,199,198,196,194,192,190,188,186,184,182,180,178,176,174,172,170,168,166,164,162,159,157,155,153,151,148,146,144,141,139,137,134,132,130,127,125,122,120,117,115,112,110,107,105,102,100,97,95,92,90,87,85,82,79,77,74,72,69,66,64,61,58,56,53,51,48,45,43,40};
  static unsigned char glow_led_states[GLOW_LED_COUNT]; // These are prescale values - set to the max you want the LED to hit during the cycle
  static unsigned char glow_led_states_hold[GLOW_LED_COUNT];
  static unsigned short glow_led_counter;
  static unsigned long glow_led_last_tick;
  static unsigned short glow_led_pace; // ms/step

  /*
  (in order of precedence)
  White                     - No Connection or Not homed
  Ready                     - Green (Motors Engaged)
  Receiving motion commands - Purple (45s since last command)
  Bed temp falling (M190)   - Blue
  Bed temp rising (M190)    - Orange
  Bed temp >= 50degC        - Red

  Blue/orange are also triggered when the target temperature or bed temperature are otherwise notable
  Short of having the desktop software send explicit commands to say "cooling begin" or "cooling end," this is OK
  */
  bool quick_change = false;
  bool ramp_down_now = false;

  unsigned long now = millis();

  float bedTemp = vone->heater.currentTemperature();
  float targetTemp = vone->heater.targetTemperature();

  if (overridingLeds()) {
    glow_led_states[0] = s_overrideLeds.r;
    glow_led_states[1] = s_overrideLeds.g;
    glow_led_states[2] = s_overrideLeds.b;
    glow_led_pace = s_overrideLeds.pace;
  } else if (bedTemp > 50.0) {
    glow_led_states[0] = 255;
    glow_led_states[1] = 0;
    glow_led_states[2] = 0;
    glow_led_pace = TEMP_PACE_CURVE;
  } else if (pending_temp_change || targetTemp > 0 || bedTemp > 40) {
    if (vone->heater.isCooling()) {
      glow_led_states[0] = 0;
      glow_led_states[1] = 0;
      glow_led_states[2] = 255;
    } else {
      glow_led_states[0] = 255;
      glow_led_states[1] = 40;
      glow_led_states[2] = 0;
    }
    glow_led_pace = TEMP_PACE_CURVE;
  } else if ((now - previous_millis_serial_rx) < (unsigned long)45*1000 && previous_millis_serial_rx) {
    glow_led_states[0] = 255;
    glow_led_states[1] = 0;
    glow_led_states[2] = 255;
    glow_led_pace = 30;
    quick_change = true;
  } else if ((now - previous_millis_serial_rx) < stepper_inactive_time && previous_millis_serial_rx) {
    glow_led_states[0] = 0;
    glow_led_states[1] = 255;
    glow_led_states[2] = 0;
    glow_led_pace = 30;
    quick_change = true;
  } else {
    glow_led_states[0] = 255;
    glow_led_states[1] = 255;
    glow_led_states[2] = 255;
    glow_led_pace = 30;
  }

  if (quick_change && memcmp(glow_led_states, glow_led_states_hold, sizeof(glow_led_states))) {
    // Go fast till we hit 0 (and glow_led_states is copied into glow_led_states_hold)
    glow_led_pace /= 10;
    ramp_down_now = true;
  }

  if ((now - glow_led_last_tick) > glow_led_pace) {
    glow_led_last_tick = now;
    if (glow_led_counter == 0) {
      // To avoid abrupt changes, we wait for zero-crossing before updating the actual state (_hold) from the input
      memcpy(glow_led_states_hold, glow_led_states, sizeof(glow_led_states));
      ramp_down_now = false; // We're at 0, don't go any further backwards...
    }
    // Wrap the counter
    glow_led_counter += ramp_down_now ? (glow_led_counter > 128 ? 1 : -1) : 1;
    if (glow_led_counter >= 256) {
      glow_led_counter = 0;
    }
    // Remap into a sine wave
    // "wow, this circuit printer's indicator LEDs follow a sine wave!" - nobody
    unsigned short glow_led_wrap = sin_lookup[glow_led_counter];
    for (unsigned char i = 0; i < GLOW_LED_COUNT; ++i) {
      analogWrite(glow_led_pins[i], (glow_led_wrap * glow_led_states_hold[i]) / 256);
    }
  }
}
