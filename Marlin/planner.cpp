/*
  planner.c - buffers movement commands and manages the acceleration profile plan
 Part of Grbl

 Copyright (c) 2009-2011 Simen Svale Skogsrud

 Grbl is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Grbl is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

/*
 Reasoning behind the mathematics in this module (in the key of 'Mathematica'):

 s == speed, a == acceleration, t == time, d == distance

 Basic definitions:

 Speed[s_, a_, t_] := s + (a*t)
 Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]

 Distance to reach a specific speed with a constant acceleration:

 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()

 Speed after a given distance of travel with constant acceleration:

 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 m -> Sqrt[2 a d + s^2]

 DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]

 When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 from initial speed s1 without ever stopping at a plateau:

 Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()

 IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "src/compensationAlgorithms/api.h"
#include "src/api/movement/movement.h" // mm to steps
#include "macros.h"

//===========================================================================
//=============================public variables ============================
//===========================================================================

unsigned long minsegmenttime;
float max_feedrate[4]; // set the max speeds
unsigned long max_acceleration_units_per_sq_second[4]; // Use M201 to override by software
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
float retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
float max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
float max_z_jerk;
float max_e_jerk;
float mintravelfeedrate;

// Parameters added by voltera.
float calib_x_scale;
float calib_y_scale;
float calib_cos_theta;
float calib_tan_theta;
float calib_x_backlash;
float calib_y_backlash;

unsigned long axis_steps_per_sqr_second[NUM_AXIS];

// The current position of the tool in absolute steps
long position[4];   //rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[4]; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment

//===========================================================================
//=================semi-private variables, used in inline  functions    =====
//===========================================================================
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now

//===========================================================================
//=============================private variables ============================
//===========================================================================

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) {
    block_index = 0;
  }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) {
    block_index = BLOCK_BUFFER_SIZE;
  }
  block_index--;
  return(block_index);
}

bool skew_adjustments_enabled = true;
void plan_enable_skew_adjustment(bool enabed) {
  skew_adjustments_enabled = enabed;
}

//===========================================================================
//=============================functions         ============================
//===========================================================================

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
FORCE_INLINE float estimate_acceleration_distance(
  float initial_rate,
  float target_rate,
  float acceleration
) {
  if (acceleration!=0) {
    return((target_rate*target_rate-initial_rate*initial_rate)/
      (2.0*acceleration));
  }
  else {
    return 0.0;  // acceleration was 0, set acceleration distance to 0
  }
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

FORCE_INLINE float intersection_distance(
  float initial_rate,
  float final_rate,
  float acceleration,
  float distance
) {
  if (acceleration!=0) {
    return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
      (4.0*acceleration) );
  }
  else {
    return 0.0;  // acceleration was 0, set intersection distance to 0
  }
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if(initial_rate <120) {
    initial_rate=120;
  }
  if(final_rate < 120) {
    final_rate=120;
  }

  long acceleration = block->acceleration_st;
  int32_t accelerate_steps =
    ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps =
    floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));

  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps,block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }

  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if(block->busy == false) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps+plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  CRITICAL_SECTION_END;
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}


// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
static void planner_reverse_pass_kernel(block_t *current, block_t *next) {
  if(!current) {
    return;
  }

  if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed,
        max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      }
      else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;

  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END

  if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    block_t *block[3] = {
      NULL, NULL, NULL         };
    while(block_index != tail) {
      block_index = prev_block_index(block_index);
      block[2]= block[1];
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[1], block[2]);
    }
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
static void planner_forward_pass_kernel(block_t *previous, block_t *current) {
  if(!previous) {
    return;
  }

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min( current->entry_speed,
      max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = {
    NULL, NULL, NULL   };

  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2]);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor for each junction. Must be called by planner_recalculate() after
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
        next->entry_speed/current->nominal_speed);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if(next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
    MINIMUM_PLANNER_SPEED/next->nominal_speed);
    next->recalculate_flag = false;
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor)
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if
//     a. The speed increase within one block would require faster accelleration than the one, true
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate() {
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  log
    << F("Motion buffer size: ")
    << BLOCK_BUFFER_SIZE << F(" moves ")
    << F("(") << sizeof(block_buffer) << F(" bytes)")
    << endl;

  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  previous_nominal_speed = 0.0;
}

inline int sign(float value) {
  return value == 0 ? 0 : (value > 0 ? 1 : -1);
}

static void s_applyCompensationAlgorithms(float& x, float& y, float& z, float& e) {
  // Axis skew compensation
  // Note: axis skew make homing non-trivial because when you are homing one axis, both axis are actually moving.
  // E.g. -Y switch could trigger when you are homing X axis.
  // This is why we allow skew compensation to be skipped
  if (skew_adjustments_enabled) {
    applySkewCompensation(x, y, calib_cos_theta, calib_tan_theta);
  } else {
    if (logging_enabled) {
      log
        << F("Skipping skew compensation for ") << x
        << F(",") << y
        << F(",") << z
        << F(",") << e
        << endl;
    }
  }

  // Axis scaling compensation
  applyScalingCompensation(x, y, calib_x_scale, calib_y_scale);
}

static void s_convertMMToSteps(float x, float y, float z, float e, long steps[]) {
  steps[ X_AXIS ] = millimetersToSteps(x, X_AXIS);
  steps[ Y_AXIS ] = millimetersToSteps(y, Y_AXIS);
  steps[ Z_AXIS ] = millimetersToSteps(z, Z_AXIS);
  steps[ E_AXIS ] = millimetersToSteps(e, E_AXIS);
}

static void s_applyBacklashCompensation(long& xSteps, long& ySteps) {
  static float s_prevMoveDirectionX = 0;
  auto directionX = sign(xSteps);
  if ( directionX != 0 && s_prevMoveDirectionX != directionX) {
    s_prevMoveDirectionX = directionX;
    auto stepOffsetX = millimetersToSteps(calib_x_backlash, X_AXIS);
    xSteps = applyBacklashCompensation("x", directionX, xSteps, stepOffsetX);
  }

  static float s_prevMoveDirectionY = 0;
  auto directionY = sign(ySteps);
  if (directionY != 0 && s_prevMoveDirectionY != directionY) {
    s_prevMoveDirectionY = directionY;
    auto stepOffsetY = millimetersToSteps(calib_y_backlash, Y_AXIS);
    ySteps = applyBacklashCompensation("y", directionY, ySteps, stepOffsetY);
  }
}

// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(float x, float y, float z, float e, float feed_rate)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head) {
    periodic_work();
  }

  // Apply compensation algorithms to compute the target position
  // NOTE: the following comment is fishy, but could save debugging time if it's true and we trip over this later
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  s_applyCompensationAlgorithms(x, y, z, e);

  // Compute the number of steps needed to reach the target
  long target[4];
  s_convertMMToSteps(x, y, z, e, target);
  long steps_x_signed = target[X_AXIS] - position[X_AXIS];
  long steps_y_signed = target[Y_AXIS] - position[Y_AXIS];
  long steps_z_signed = target[Z_AXIS] - position[Z_AXIS];
  long steps_e_signed = target[E_AXIS] - position[E_AXIS];

  // Apply backlash compensation
  s_applyBacklashCompensation(steps_x_signed, steps_y_signed);

  // Prepare a new block
  block_t *block = &block_buffer[block_buffer_head];

  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

  block->steps_x = labs(steps_x_signed);
  block->steps_y = labs(steps_y_signed);
  block->steps_z = labs(steps_z_signed);

  block->steps_e = labs(steps_e_signed);
  block->steps_e *= volumetric_multiplier;
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100;

  block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments)
  {
    return;
  }

  // Compute direction bits for this block
  block->direction_bits = 0;

  if (steps_x_signed < 0) {
    block->direction_bits |= (1<<X_AXIS);
  }
  if (steps_y_signed < 0) {
    block->direction_bits |= (1<<Y_AXIS);
  }
  if (steps_z_signed < 0) {
    block->direction_bits |= (1<<Z_AXIS);
  }
  if (steps_e_signed < 0) {
    block->direction_bits |= (1<<E_AXIS);
  }

  // enable active axes
  if (block->steps_x != 0) { enable_x(); }
  if (block->steps_y != 0) { enable_y(); }
  if (block->steps_z != 0) { enable_z(); }
  if (block->steps_e != 0) { enable_e(); }

  if (block->steps_e == 0) {
    if (feed_rate < mintravelfeedrate) {
      feed_rate = mintravelfeedrate;
    }
  } else {
    if (feed_rate < minimumfeedrate) {
      feed_rate = minimumfeedrate;
    }
  }

  // Delta_mm is the mm that our axis will actually move to meet the global coordinates.
  float delta_mm[4];
  delta_mm[X_AXIS] = stepsToMillimeters(steps_x_signed, X_AXIS);
  delta_mm[Y_AXIS] = stepsToMillimeters(steps_y_signed, Y_AXIS);
  delta_mm[Z_AXIS] = stepsToMillimeters(steps_z_signed, Z_AXIS);
  delta_mm[E_AXIS] = stepsToMillimeters(steps_e_signed, E_AXIS) * volumetric_multiplier * extrudemultiply / 100.0;

  if (
    block->steps_x <= dropsegments &&
    block->steps_y <= dropsegments &&
    block->steps_z <= dropsegments
  ) {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  } else {
    block->millimeters = sqrt(
      square(delta_mm[X_AXIS]) +
      square(delta_mm[Y_AXIS]) +
      square(delta_mm[Z_AXIS])
    );
  }
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides

    // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);

  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[4];
  float speed_factor = 1.0; //factor <=1 do decrease speed

  for (auto i = 0u; i < 4; ++i) {
    current_speed[i] = delta_mm[i] * inverse_second;

    // log
    //   << F(" Speed: ") << current_speed[i]
    //   << F(", maxF: ") << max_feedrate[i]
    //   << endl;

    if(fabs(current_speed[i]) > max_feedrate[i]) {
      speed_factor = min(
        speed_factor,
        max_feedrate[i] / fabs(current_speed[i])
      );
    }
  }

  // log
  //   << " x=" << x
  //   << " y=" << y
  //   << " z=" << z
  //   << " nr=" << block->nominal_rate
  //   << "steps/s sf=" << speed_factor
  //   << " sec=" << block->step_event_count
  //   << " " << block->millimeters
  //   << "mm f=" << feed_rate
  //   << endl;

  // Correct the speed
  if (speed_factor < 1.0) {
    for (auto i = 0u; i < 4; ++i) {
      current_speed[i] *= speed_factor;

      // log
      //   << F(" Corrected Speed:") << current_speed[i]
      //   << F(", factor:") << speed_factor
      //   << endl;
    }
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  float steps_per_mm = block->step_event_count/block->millimeters;
  if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
  {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else
  {
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    // Limit acceleration per axis
    if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
  }
  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (F_CPU / 8.0)));

  // Start with a safe speed
  float vmax_junction = max_xy_jerk/2;
  float vmax_junction_factor = 1.0;
  if(fabs(current_speed[Z_AXIS]) > max_z_jerk/2)
    vmax_junction = min(vmax_junction, max_z_jerk/2);
  if(fabs(current_speed[E_AXIS]) > max_e_jerk/2)
    vmax_junction = min(vmax_junction, max_e_jerk/2);

  vmax_junction = min(vmax_junction, block->nominal_speed);

  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((current_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
    //    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk/jerk);
    }
    if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor= min(vmax_junction_factor, (max_z_jerk/fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    }


    if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk/fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
    }
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed

  }
  block->max_entry_speed = vmax_junction;



  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // log
  //   << F(" max entry speed") << block->max_entry_speed
  //   << F(", nominal speed ") << block->nominal_speed
  //   << F(", entrySpeed ") << block->entry_speed
  //   << end;

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) {
    block->nominal_length_flag = true;
  }
  else {
    block->nominal_length_flag = false;
  }
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
  previous_nominal_speed = block->nominal_speed;


  calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed,
  safe_speed/block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]

  planner_recalculate();
}

void plan_set_position(float x, float y, float z, float e) {
  if (logging_enabled) {
    log
      << F("Resetting planner position to")
      << F(" X:") << x
      << F(" Y:") << y
      << F(" Z:") << z
      << F(" E:") << e
      << endl;
  }

  // Apply compensation algorithms to compute the new position in steps
  // Note: we don't need to apply backlash compensation because we are not moving
  s_applyCompensationAlgorithms(x, y, z, e);
  s_convertMMToSteps(x, y, z, e, position);

  // Set stepper position
  st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]);
  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
}

void plan_set_e_position(float e) {
  log << F("Resetting planner's E position to E:") << e << endl;
  position[E_AXIS] = millimetersToSteps(e, E_AXIS);
  st_set_e_position(position[E_AXIS]);
}

uint8_t movesplanned() {
  return (block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
}

// Calculate the steps/s^2 acceleration rates, based on the mm/s^s
void reset_acceleration_rates() {
  axis_steps_per_sqr_second[X_AXIS] = millimetersToSteps(max_acceleration_units_per_sq_second[X_AXIS], X_AXIS);
  axis_steps_per_sqr_second[Y_AXIS] = millimetersToSteps(max_acceleration_units_per_sq_second[Y_AXIS], Y_AXIS);
  axis_steps_per_sqr_second[Z_AXIS] = millimetersToSteps(max_acceleration_units_per_sq_second[Z_AXIS], Z_AXIS);
  axis_steps_per_sqr_second[E_AXIS] = millimetersToSteps(max_acceleration_units_per_sq_second[E_AXIS], E_AXIS);
}
