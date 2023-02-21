/*
 * planner.h
 *
 *  Created on: 2020年10月27日
 *      Author: Administrator
 */

#ifndef SRC_MOTION_PLANNER_H_
#define SRC_MOTION_PLANNER_H_

#define ENABLE_ISRS()  __enable_irq()
#define DISABLE_ISRS() __disable_irq()

#define  FORCE_INLINE  __attribute__((always_inline)) inline

#define STEPPER_TIMER_RATE 2000000
#define MINIMAL_STEP_RATE 12
#define MINIMAL_STEPVELOCITY 100.0

#define S_CURVE_ACCELERATION 1
#define DIRECT_STEPPING 0

  #define NOLESS(v, n) \
    do{ \
      if (n > v) v = n; \
    }while(0)

  #define NOMORE(v, n) \
    do{ \
      if (n < v) v = n; \
    }while(0)

  #define LIMIT(v, n1, n2) \
    do{ \
      if (n1 > v) v = n1; \
      else if (n2 < v) v = n2; \
    }while(0)

#define MIN_2(a,b)      ((a)<(b)?(a):(b))
#define MAX_2(a,b)      ((a)>(b)?(a):(b))

typedef struct block_t {

  volatile uint8_t flag;                    // Block flags (See BlockFlag enum above) - Modified by ISR and main thread!

  // Fields used by the motion planner to manage acceleration
  float nominal_speed_sqr,                  // The nominal speed for this block in (mm/sec)^2
        entry_speed_sqr,                    // Entry speed at previous-current junction in (mm/sec)^2
        max_entry_speed_sqr,                // Maximum allowable junction entry speed in (mm/sec)^2
        millimeters,                        // The total travel of this block in mm
        acceleration;                       // acceleration mm/sec^2

  union {
    uint32_t steps;                     // Step count along each axis
    uint32_t position;                   // New position to force when this sync block is executed
  };
  uint32_t step_event_count;                // The number of step events required to complete this block

  // Settings for the trapezoid generator
  uint32_t accelerate_until,                // The index of the step event on which to stop acceleration
           decelerate_after;                // The index of the step event on which to start decelerating

  #if S_CURVE_ACCELERATION == 1
    uint32_t cruise_rate,                   // The actual cruise rate to use, between end of the acceleration phase and start of deceleration phase
             acceleration_time,             // Acceleration time and deceleration time in STEP timer counts
             deceleration_time,
             acceleration_time_inverse,     // Inverse of acceleration and deceleration periods, expressed as integer. Scale depends on CPU being used
             deceleration_time_inverse;
  #else
    uint32_t acceleration_rate;             // The acceleration rate used for acceleration calculation
  #endif

  uint8_t direction_bits;                   // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

  uint32_t nominal_rate,                    // The nominal step rate for this block in step_events/sec
           initial_rate,                    // The jerk-adjusted step rate at start of block
           final_rate,                      // The minimal rate at exit
           acceleration_steps_per_s2;       // acceleration steps/sec^2

  #if DIRECT_STEPPING == 1
    page_idx_t page_idx;                    // Page index used for direct stepping
  #endif

} block_t;

void calculate_trapezoid_for_block(block_t* const block, float entry_factor, float exit_factor);

#endif /* SRC_MOTION_PLANNER_H_ */
