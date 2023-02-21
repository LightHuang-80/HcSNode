/*
 * planner.c
 *
 *  Created on: 2020年10月27日
 *      Author: Administrator
 */


/*
 * Buffer movement commands and manage the acceleration profile plan
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis.
 *
 *
 * Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 *
 * s == speed, a == acceleration, t == time, d == distance
 *
 * Basic definitions:
 *   Speed[s_, a_, t_] := s + (a*t)
 *   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 *
 * Distance to reach a specific speed with a constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 *   d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 *
 * Speed after a given distance of travel with constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 *   m -> Sqrt[2 a d + s^2]
 *
 * DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 *
 * When to start braking (di) to reach a specified destination speed (s2) after accelerating
 * from initial speed s1 without ever stopping at a plateau:
 *   Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 *   di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 *
 * IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 *
 *
 */

/*
 *  Block speed basic
 *                             PLANNER SPEED DEFINITION
                                     +--------+   <- current->nominal_speed
                                    /          \
         current->entry_speed ->   +            \
                                   |             + <- next->entry_speed (aka exit speed)
                                   +-------------+
                                       time -->
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "planner.h"

static float estimate_acceleration_distance(const float initial_rate, const float target_rate, const float accel) {
      if (accel == 0) return 0; // accel was 0, set acceleration distance to 0
      return (target_rate*target_rate - initial_rate*initial_rate) / (accel * 2);
    }

static float intersection_distance(const float initial_rate, const float final_rate, const float accel, const float distance) {
  if (accel == 0) return 0; // accel was 0, set intersection distance to 0
  return (accel * 2 * distance - initial_rate*initial_rate + final_rate*final_rate) / (accel * 4);
}

static float final_speed(const float initial_velocity, const float accel, const float distance) {
  return sqrtf(initial_velocity*initial_velocity + 2 * accel * distance);
}

static uint32_t get_period_inverse(const uint32_t d) {
      return d ? 0xFFFFFFFF / d : 0xFFFFFFFF;
    }

void calculate_trapezoid_for_block(block_t* const block, float entry_factor, float exit_factor)
{
  uint32_t initial_rate = ceilf(block->nominal_rate * entry_factor),
	       final_rate = ceilf(block->nominal_rate * exit_factor); // (steps per second)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  NOLESS(initial_rate, (uint32_t)(MINIMAL_STEP_RATE));
  NOLESS(final_rate, (uint32_t)(MINIMAL_STEP_RATE));

  #if S_CURVE_ACCELERATION == 1
    uint32_t cruise_rate = initial_rate;
  #endif

  const int32_t accel = block->acceleration_steps_per_s2;

  // Steps required for acceleration, deceleration to/from nominal rate
  uint32_t accelerate_steps = ceilf(estimate_acceleration_distance(initial_rate, block->nominal_rate, accel)),
           decelerate_steps = floorf(estimate_acceleration_distance(block->nominal_rate, final_rate, -accel));

  // Steps between acceleration and deceleration, if any
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Does accelerate_steps + decelerate_steps exceed step_event_count?
  // Then we can't possibly reach the nominal rate, there will be no cruising.
  // Use intersection_distance() to calculate accel / braking time in order to
  // reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
     const float accelerate_steps_float = ceilf(intersection_distance(initial_rate, final_rate, accel, block->step_event_count));
     accelerate_steps = MIN_2((uint32_t)(MAX_2(accelerate_steps_float, 0)), block->step_event_count);
     plateau_steps = 0;

     #if S_CURVE_ACCELERATION == 1
       // We won't reach the cruising rate. Let's calculate the speed we will reach
       cruise_rate = final_speed(initial_rate, accel, accelerate_steps);
     #endif
   }
   #if S_CURVE_ACCELERATION == 1
     else // We have some plateau time, so the cruise rate will be the nominal rate
       cruise_rate = block->nominal_rate;
   #endif

#if S_CURVE_ACCELERATION == 1
  // Jerk controlled speed requires to express speed versus time, NOT steps
  uint32_t acceleration_time = ((float)(cruise_rate - initial_rate) / accel) * (STEPPER_TIMER_RATE),
           deceleration_time = ((float)(cruise_rate - final_rate) / accel) * (STEPPER_TIMER_RATE),

  // And to offload calculations from the ISR, we also calculate the inverse of those times here
           acceleration_time_inverse = get_period_inverse(acceleration_time),
           deceleration_time_inverse = get_period_inverse(deceleration_time);
#endif

  // Store new block parameters
  block->accelerate_until = accelerate_steps;
  block->decelerate_after = accelerate_steps + plateau_steps;
  block->initial_rate = initial_rate;

#if S_CURVE_ACCELERATION == 1
  block->acceleration_time = acceleration_time;
  block->deceleration_time = deceleration_time;
  block->acceleration_time_inverse = acceleration_time_inverse;
  block->deceleration_time_inverse = deceleration_time_inverse;
  block->cruise_rate = cruise_rate;
#endif
  block->final_rate = final_rate;
}
