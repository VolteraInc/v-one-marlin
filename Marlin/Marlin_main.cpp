/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "version.h"
#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "temperature_profile.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"

#include "src/api/api.h"
#include "src/commands/processing.h"

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
int extruder_multiply[EXTRUDERS] = {100};
float volumetric_multiplier[EXTRUDERS] = {1.0};
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
float zprobe_zoffset;

uint8_t active_extruder = 0;
int fanSpeed=0;

bool glow_led_override = false;
bool glow_force_green = false; // For taking pictures of the printer without a PC attached.
bool override_p_bot = false;
float min_z_x_pos;
float min_z_y_pos;
float xypos_x_pos;
float xypos_y_pos;
char product_serial_number[15];
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

bool logging_enabled = false;

//===========================================================================
//=============================Private Variables=============================
//===========================================================================

//Inactivity shutdown variables
static unsigned long previous_millis_serial_rx = 0;

// After this long without serial traffic *and* no movement, everything shuts down
static unsigned long heater_inactive_time = DEFAULT_HEATER_DEACTIVE_TIME * 1000l;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME * 1000l;

unsigned long starttime = 0;
unsigned long stoptime = 0;


bool Stopped = false;


//===========================================================================
//=============================Routines======================================
//===========================================================================

void checkBufferEmpty();
void manage_inactivity();

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

extern "C" {
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

void setup() {
  MYSERIAL.begin(BAUDRATE);

  // sometimes there are noise characters in the first message,
  // so we make the first message a bit more distinct by including dashes
  SERIAL_PROTOCOLLNPGM("--start--");

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu) {
    SERIAL_ECHO_START;
    if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
    if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
    if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
    if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
    if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  }
  MCUSR=0;

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  SERIAL_PROTOCOLPGM("firmwareVersionReport: ");
  SERIAL_PROTOCOLPGM(VERSION_STRING);
  SERIAL_PROTOCOLPGM("\n");

  sendHomedStatusUpdate();
  sendToolStatusUpdate();

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  Config_RetrieveCalibration();

  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!

  glow_force_green = READ_PIN(Z_MIN);
}

void outputBedTemperatureUpdate(float current, float target, float timeRemaining) {
  SERIAL_PROTOCOLPGM("bedTemperatureUpdate");
  SERIAL_PROTOCOLPGM(" current:"); SERIAL_PROTOCOL_F(current,1);
  SERIAL_PROTOCOLPGM(" target:"); SERIAL_PROTOCOL_F(target,1);
  SERIAL_PROTOCOLPGM(" timeRemaining:"); SERIAL_PROTOCOL_F(timeRemaining,1);
  SERIAL_PROTOCOLPGM("\n");
}

void periodic_output() {
  static struct {
    float position[NUM_AXIS];
    struct { float current; float target; } temperature;
  } prev; // Previously reported values
  static auto nextOutputAt = millis();

  const auto now = millis();
  if (now >= nextOutputAt) {

    // Schedule next output
    nextOutputAt += 1000;

    // Output position on change
    if (memcmp(prev.position, current_position, sizeof(prev.position)) != 0) {
      memcpy(prev.position, current_position, sizeof(prev.position));
      SERIAL_PROTOCOLPGM("positionUpdate");
      SERIAL_PROTOCOLPGM(" x:"); SERIAL_PROTOCOL_F(current_position[X_AXIS], 6);
      SERIAL_PROTOCOLPGM(" y:"); SERIAL_PROTOCOL_F(current_position[Y_AXIS], 6);
      SERIAL_PROTOCOLPGM(" z:"); SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
      SERIAL_PROTOCOLPGM(" e:"); SERIAL_PROTOCOL_F(current_position[E_AXIS], 6);
      SERIAL_PROTOCOLPGM("\n");
    }


    const auto current = degBed();
    const auto target = degTargetBed();
    const auto timeRemaining = profile_remaining_time();

    // Output temperature if temp or target changes or we are running a heating profile
    // NOTE: The temp sensor is noisy so filter small changes.
    bool tempChanged = abs(prev.temperature.current - current) >= 0.5;
    bool targetChanged = prev.temperature.target != target;
    if ( !profile_empty() || tempChanged || targetChanged) {
        if (targetChanged) {
          // Output an extra data point to signal the change in target temp
          // otherwise you will have a confusing sloped line from the previous
          // temp update to this one.
          // NOTE: sending a 0 for time remaining -- not sure what else to send
          // and this is better than the current time remaining of what may be
          // a different profile.
          outputBedTemperatureUpdate(current, prev.temperature.target, 0);
        }
        outputBedTemperatureUpdate(current, target, timeRemaining);

        prev.temperature.current = current;
        prev.temperature.target = target;
    }
  }
}

static void s_checkForEndstopHits() {
  bool triggered[3];
  long stepsWhenTriggered[3];
  if (readAndResetEndstops(triggered, stepsWhenTriggered)) {
    // Reset homing and tool preparations
    // Note: We might not need to reset all the axes, but it's more robust to do so.
    setHomedState(X_AXIS, 0);
    setHomedState(Y_AXIS, 0);
    setHomedState(Z_AXIS, 0);
    resetToolPreparations();

    // Output an error
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to complete movement, limit switch triggered. Motion stopped at");
    if (triggered[ X_AXIS ]) {
      SERIAL_ERRORPGM(" x:"); SERIAL_ERROR((float)stepsWhenTriggered[X_AXIS] / axis_steps_per_unit[X_AXIS]);
    }
    if (triggered[ Y_AXIS ]) {
      SERIAL_ERRORPGM(" y:"); SERIAL_ERROR((float)stepsWhenTriggered[Y_AXIS] / axis_steps_per_unit[Y_AXIS]);
    }
    if (triggered[ Z_AXIS ]) {
      SERIAL_ERRORPGM(" z:"); SERIAL_ERROR((float)stepsWhenTriggered[Z_AXIS] / axis_steps_per_unit[Z_AXIS]);
    }
    SERIAL_ECHOPGM("\n");
  }
}

void read_commands() {
  static char command[MAX_CMD_SIZE];
  static int index = 0;

  while (MYSERIAL.available() > 0 && !command_queue.full()) {
    char ch = MYSERIAL.read();

    // End of command
    if (ch == '\n' || ch == '\r') {
      // Handle empty lines
      if(index == 0 || index >= (MAX_CMD_SIZE - 1)) {
        // We output a message, because this is a symptom that something is wrong
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Received a blank command, ignoring");
      } else {
        // Add to command queue
        command[index] = 0; // terminate string
        command_queue.push(command);
      }

      // Reset the write position for the next command
      index = 0;

    // Command too long
    } else if (index >= (MAX_CMD_SIZE - 1)) {
      command[index] = 0; // terminate string (so we can include it in the error)
      index = 0; // reset write index
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM("Unable to process command, command is too long, will ignore until end of command found --");
      SERIAL_ERRORLN(command);

    // Add character to command
    } else {
      command[index++] = ch;
    }
  }
}

void process_command() {
  if (command_prefix_seen('V')) {
    process_vcode((int)code_value());
  } else if (command_prefix_seen('D')) {
    process_dcode((int)code_value());
  } else if (command_prefix_seen('I')) {
    process_icode((int)code_value());
  } else if(command_prefix_seen('G')) {
    process_gcode((int)code_value());
  } else if(command_prefix_seen('M')) {
    process_mcode((int)code_value());
  } else {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(command_queue.front());
    SERIAL_ECHOLNPGM("\"");
  }
}

void periodic_work() {
  manage_heating_profile();
  manage_heater();
  manage_inactivity();
  handle_glow_leds();
  s_checkForEndstopHits();
  checkBufferEmpty();
  periodic_output();
}

void loop() {
  read_commands();

  if(!command_queue.empty()) {
    // Refresh the timeouts before processing so that that we have
    // the entire timeout duration to process the command
    // Note: this is necessary because some commands block
    // (aka busy-wait) but will still check for inactivity
    // (ie. they all call periodic_work())
    previous_millis_serial_rx = millis();
    refresh_cmd_timeout();

    process_command();
    command_queue.pop();

    // Send Acknowledgement
    SERIAL_PROTOCOLLNPGM(MSG_OK);

    // Refresh the timeouts after processing so that the user/sw
    // has then entire timeout duration to issue another command
    previous_millis_serial_rx = millis();
    refresh_cmd_timeout();
  }

  periodic_work();
}

void manage_inactivity() {
  static auto nextCheckAt = millis();

  const auto now = millis();
  if (now >= nextCheckAt) {
    // Schedule next check
    nextCheckAt += 1000;

    if((now - previous_millis_active_cmd) > stepper_inactive_time && stepper_inactive_time) {
      if(!blocks_queued()) {
        refresh_cmd_timeout(); // Reseting timeout stops us from constantly checking.
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("The stepper has been inactive for more than "); SERIAL_ECHO(stepper_inactive_time);
        SERIAL_ECHOPGM("ms, deactivating motors\n");
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        resetToolPreparations();
      }
    }

    // If we've been waiting for ~1 hour. Kill the heater.
    if(now - previous_millis_serial_rx > heater_inactive_time && heater_inactive_time) {
      previous_millis_serial_rx = now; //Resetting timeout stops us from constantly checking
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("No communication for more than "); SERIAL_ECHO(heater_inactive_time);
      SERIAL_ECHOPGM("ms, deactivating heater\n");
      disable_heater();
    }
  }
}

void checkBufferEmpty() {
  static uint8_t buffer_fill = 0;
  uint8_t new_buffer_fill = movesplanned();
  if (buffer_fill && !new_buffer_fill) {
    SERIAL_PROTOCOLLNPGM("empty");
  }
  buffer_fill = new_buffer_fill;
}

void handle_glow_leds() {
  #define GLOW_LED_COUNT 3
  #define TEMP_PACE_CURVE max((-5 * bedTemp) / 42 + 36, 5)
  static const char glow_led_pins[GLOW_LED_COUNT] = {LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN};
  static const unsigned char sin_lookup[256] = {43,45,48,51,53,56,58,61,64,66,69,72,74,77,79,82,85,87,90,92,95,97,100,102,105,107,110,112,115,117,120,122,125,127,130,132,134,137,139,141,144,146,148,151,153,155,157,159,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190,192,194,196,198,199,201,203,205,206,208,209,211,213,214,216,217,219,220,222,223,224,226,227,228,230,231,232,233,234,235,237,238,239,240,241,242,242,243,244,245,246,246,247,248,249,249,250,250,251,251,252,252,253,253,253,254,254,254,254,255,255,255,255,255,255,255,255,255,255,255,254,254,254,254,253,253,253,252,252,251,251,250,250,249,249,248,247,246,246,245,244,243,242,242,241,240,239,238,237,235,234,233,232,231,230,228,227,226,224,223,222,220,219,217,216,214,213,211,209,208,206,205,203,201,199,198,196,194,192,190,188,186,184,182,180,178,176,174,172,170,168,166,164,162,159,157,155,153,151,148,146,144,141,139,137,134,132,130,127,125,122,120,117,115,112,110,107,105,102,100,97,95,92,90,87,85,82,79,77,74,72,69,66,64,61,58,56,53,51,48,45,43,40};
  static unsigned char glow_led_states[GLOW_LED_COUNT]; // These are prescale values - set to the max you want the LED to hit during the cycle
  static unsigned char glow_led_states_hold[GLOW_LED_COUNT];
  static unsigned short glow_led_counter;
  static unsigned long glow_led_last_tick;
  static unsigned short glow_led_pace; // ms/step
  if (glow_led_override) {
    // Set this to 0 so we start from 0 when glow_led_override is unset
    glow_led_counter = 0;
    return;
  }

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

  float bedTemp = degBed();
  float targetTemp = degTargetBed();
  if(bedTemp > 50.0){
    glow_led_states[0] = 255;
    glow_led_states[1] = 0;
    glow_led_states[2] = 0;
    glow_led_pace = TEMP_PACE_CURVE;
  } else if (pending_temp_change || targetTemp > 0 || bedTemp > 40) {
    if (isCoolingBed()) {
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
  } else if (glow_force_green || ((now - previous_millis_serial_rx) < stepper_inactive_time && previous_millis_serial_rx)) {
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

void kill() {
  cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();

  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop() {
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
  }
}

bool IsStopped() {
  return Stopped;
};

void setStepperInactiveDuration(unsigned long duration) {
  stepper_inactive_time = duration;
}
