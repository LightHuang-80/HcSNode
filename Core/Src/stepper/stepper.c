/*
 * stepper.c
 *
 *  Created on: 2020年8月20日
 *      Author: Administrator
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "tim.h"

#include "stepper.h"
#include "planner.h"
#include "scurve.h"
#include "interpolation.h"

#if !defined (Stepper_Dir_Pin)
  #define Stepper_Dir_Pin GPIO_PIN_9
  #define Stepper_Dir_GPIO_Port GPIOC
#endif

#define HAL_TIMER_TYPE_MAX 0xFFFF

#define RUN_Mode_Normal        0x01
#define RUN_Mode_StepsControl  0x02
#define RUN_Mode_AngleControl  0x03

typedef void (*Stepper_Motion_Finished_CB)(int8_t type);

extern uint16_t g_AbsAngle;

uint32_t delta_error;
uint32_t acceleration_time, deceleration_time;
uint8_t  steps_per_isr;
bool     abort_current_block;
uint32_t step_events_completed = 0, // The number of step events executed in the current block
         accelerate_until,          // The count at which to stop accelerating
         decelerate_after,          // The count at which to start decelerating
         step_event_count;          // The total event count for the current block
uint8_t  last_direction_bits;
block_t* current_block;
bool     bezier_2nd_half;
int32_t  ticks_nominal = -1;
uint8_t  oversampling_factor = 0;

typedef struct STEPPER_RUN_DATA{
	int32_t targetPos;
	int32_t currentPos;

	int32_t steps;

	uint16_t speed;     // steps per one second
	uint8_t  micros;    // 细分
	int8_t   dir;       // 方向

	uint32_t acce;		// accelerate
	uint32_t dece;		// decelerate

	uint16_t state;
	uint16_t mode;

	uint16_t targetAngle;
	uint16_t currentAngle;

	void*    motion;
	Stepper_Motion_Finished_CB motionFinishedCB;
}Stepper_Run_Data_t;

Stepper_Run_Data_t  g_StepperData;


static void discard_current_block() {
  current_block = NULL;

  acceleration_time = 0;
  deceleration_time = 0;

  if (g_StepperData.motionFinishedCB != NULL){
	// Interpolation motion finished
	g_StepperData.motionFinishedCB(SCURVE_MOTION_TYPE);
	g_StepperData.motionFinishedCB = NULL;
  }
}

static void discard_interpolation_block() {
  g_InterpolationBlock.enable = 0;

  if (g_StepperData.motionFinishedCB != NULL){
	// Interpolation motion finished
	g_StepperData.motionFinishedCB(INTERPOLATION_MOTION_TYPE);
	g_StepperData.motionFinishedCB = NULL;
  }
}

void Stepper_Init(uint8_t microsteps)
{
	g_StepperData.micros = microsteps;
	g_StepperData.dir    = STEPPER_Dir_CW;  // clockwise

	g_StepperData.mode = RUN_Mode_Normal;
	g_StepperData.state = STEPPER_State_Stop;

	g_StepperData.currentPos = 0;
	g_StepperData.targetPos = 0;

	g_StepperData.currentAngle = 0;
	g_StepperData.targetAngle = 0;

	g_StepperData.steps = 0;

	g_StepperData.speed = 400;  // default setting 1 round per second
	g_StepperData.acce  = 4 * g_StepperData.speed;  // 4x speed
	g_StepperData.dece  = 8 * g_StepperData.speed;  // 8x speed

	g_StepperData.motion = NULL;
	g_StepperData.motionFinishedCB = NULL;

	Stepper_SetSpeed(g_StepperData.speed);
	Stepper_SetDir(g_StepperData.dir);
}

void Stepper_ChangeDir()
{
	if (g_StepperData.dir == STEPPER_Dir_CW)
		g_StepperData.dir = STEPPER_Dir_CCW;
	else
		g_StepperData.dir = STEPPER_Dir_CW;

	HAL_GPIO_TogglePin(Stepper_Dir_GPIO_Port, Stepper_Dir_Pin);
}

int8_t Stepper_GetDir()
{
	return g_StepperData.dir;
}

void Stepper_SetDir(uint32_t dir)
{
	if (dir == STEPPER_Dir_CW) {
		g_StepperData.dir = dir;
		HAL_GPIO_WritePin(Stepper_Dir_GPIO_Port, Stepper_Dir_Pin, GPIO_PIN_SET);
	}else if (dir == STEPPER_Dir_CCW) {
		g_StepperData.dir = dir;
		HAL_GPIO_WritePin(Stepper_Dir_GPIO_Port, Stepper_Dir_Pin, GPIO_PIN_RESET);
	}

    last_direction_bits = dir;
}

void Stepper_SetSpeed(uint16_t speed)
{
	if (speed == 0) return;
	if (speed >= 320000) return;

	/*Speed calc, steps(speed) per 1 second*/
	g_StepperData.speed = speed;

	uint32_t prescaler = 2000000; // 2M tim1 clock
	uint32_t period = (prescaler / speed) - 1;
	htim1.Instance->ARR = period;
	htim1.Instance->CCR1 = period / 2;  // pulse width
}

uint16_t Stepper_GetSpeed()
{
	return g_StepperData.speed;
}

void Stepper_RunSteps(int32_t steps)
{
	vPortEnterCritical();
	g_StepperData.steps = steps;
	vPortExitCritical();

	if (steps >= 0){
		Stepper_SetDir(STEPPER_Dir_CW);
	}else{
		Stepper_SetDir(STEPPER_Dir_CCW);
	}

	g_StepperData.mode  = RUN_Mode_StepsControl;
	g_StepperData.state = STEPPER_State_Running;
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
}

void Stepper_GoHome()
{
	int32_t steps = 0 - g_StepperData.currentPos;
	Stepper_RunSteps(steps);
}

void Stepper_Stop()
{
	g_StepperData.state = STEPPER_State_Stop;
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);

	//Clear interpolation and scurve
	discard_interpolation_block();
	discard_current_block();
}

void Stepper_Run()
{
	g_StepperData.mode  = RUN_Mode_Normal;
	g_StepperData.state = STEPPER_State_Running;
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
}

void Stepper_RunMotion(Stepper_Motion_Finished_CB cb)
{
	g_StepperData.motionFinishedCB = cb;
	Stepper_Run();
}

uint16_t Stepper_GetState()
{
	return g_StepperData.state;
}

/* PID control
 * */

typedef struct DC_PID_Data{
	float KP;
	float KI;
	float KD;

	float sumerr;
	float lasterr;

	int32_t preverr;
}DC_PID_t;

DC_PID_t DcPid0 = {0.3f, 0.15f, 0.15f, 0, 0, 0};

static void DC_UpdateIncPIDPos()
{
	float err = g_StepperData.targetPos - g_StepperData.currentPos;

	int32_t incPos = DcPid0.KP * (err - DcPid0.preverr);
	        incPos += DcPid0.KI * err;
	        incPos += DcPid0.KD * (err - 2.0f * DcPid0.lasterr + DcPid0.preverr);

	DcPid0.preverr = DcPid0.lasterr;
	DcPid0.lasterr = err;

	Stepper_RunSteps(incPos);
}

static void DC_UpdateAnglePID()
{
	// Update the global absolute angle
	//g_StepperData.currentAngle = g_AbsAngle;
	g_StepperData.currentAngle = 0;

	float err = g_StepperData.targetAngle - g_StepperData.currentAngle;

	int32_t incPos = DcPid0.KP * (err - DcPid0.preverr);
	        incPos += DcPid0.KI * err;
	        incPos += DcPid0.KD * (err - 2.0f * DcPid0.lasterr + DcPid0.preverr);

	DcPid0.preverr = DcPid0.lasterr;
	DcPid0.lasterr = err;

	if (err < 0){
		Stepper_SetDir(STEPPER_Dir_CW);
	}else{
		Stepper_SetDir(STEPPER_Dir_CCW);
	}

	if (err <= 10.0 && err >= -10.0){
		Stepper_Stop();
	}
}

void Stepper_Process(int32_t currentPos)
{
	if (g_StepperData.mode == RUN_Mode_StepsControl){

		// Save the currentPos
		vPortEnterCritical();
		g_StepperData.currentPos = currentPos;
		vPortExitCritical();

		DC_UpdateIncPIDPos();
	}
}

void Stepper_RunAngle(uint16_t angle)
{
	g_StepperData.mode = RUN_Mode_AngleControl;

	float fangle = angle * 1.0;
	float target = fangle * 16384.0 / 360.0; // AS5048 resolution per 1 degree

	vPortEnterCritical();
	g_StepperData.currentAngle = 0;
	g_StepperData.targetAngle = (uint16_t)target;
	vPortExitCritical();

	g_StepperData.state = STEPPER_State_Running;
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
}

static uint32_t calc_timer_interval(uint32_t step_rate, uint8_t* loops) {
    uint32_t timer;

    // Scale the frequency, as requested by the caller
    step_rate <<= oversampling_factor;

    uint8_t multistep = 1;
    *loops = multistep;

     // In case of high-performance processor, it is able to calculate in real-time
     timer = (uint32_t)(STEPPER_TIMER_RATE) / step_rate;
     return timer;
}

uint32_t Stepper_PushBlock(block_t* block)
{
    // No acceleration / deceleration time elapsed so far
    acceleration_time = deceleration_time = 0;

    uint8_t oversampling = 0;

    // Based on the oversampling factor, do the calculations
    step_event_count = block->step_event_count << oversampling;

    // Initialize Bresenham delta errors to 1/2
    delta_error = -(int32_t)(step_event_count);

    // No step events completed so far
    step_events_completed = 0;

    // Compute the acceleration and deceleration points
    accelerate_until = block->accelerate_until << oversampling;
    decelerate_after = block->decelerate_after << oversampling;

    if (block->direction_bits != last_direction_bits) {

      Stepper_SetDir(block->direction_bits);
    }

    // Mark the time_nominal as not calculated yet
    ticks_nominal = -1;

#if S_CURVE_ACCELERATION == 1
    // Initialize the Bézier speed curve
    calc_bezier_curve_coeffs(block->initial_rate, block->cruise_rate, block->acceleration_time_inverse);
    // We haven't started the 2nd half of the trapezoid
    bezier_2nd_half = false;
#else
    // Set as deceleration point the initial rate of the block
    acc_step_rate = current_block->initial_rate;
#endif

    // Initialize the normal parameters
    abort_current_block = false;
    steps_per_isr = 1;

    // Calculate the initial timer interval
    uint32_t interval = calc_timer_interval(block->initial_rate, &steps_per_isr);

    current_block = block;
    return interval;
}

/**
 * This phase of the ISR should ONLY create the pulses for the steppers.
 * This prevents jitter caused by the interval between the start of the
 * interrupt and the start of the pulses. DON'T add any logic ahead of the
 * call to this method that might cause variation in the timing. The aim
 * is to keep pulse timing as regular as possible.
 */
static void pulse_phase_isr() {

  // If we must abort the current block, do so!
  if (abort_current_block) {
    abort_current_block = false;
    if (current_block) discard_current_block();
  }

  // If there is no current block, do nothing
  if (!current_block) return;

  // Count of pending loops and events for this iteration
  const uint32_t pending_events = step_event_count - step_events_completed;
  uint8_t events_to_do = MIN_2(pending_events, steps_per_isr);

  // Just update the value we will get at the end of the loop
  step_events_completed += events_to_do;
}

// This is the last half of the stepper interrupt: This one processes and
// properly schedules blocks from the planner. This is executed after creating
// the step pulses, so it is not time critical, as pulses are already done.

static uint32_t block_phase_isr() {

  // If no queued movements, just wait 1ms for the next block
  uint32_t interval = (STEPPER_TIMER_RATE) / 1000UL;

  // If there is a current block
  if (current_block) {

    // If current block is finished, reset pointer and finalize state
    if (step_events_completed >= step_event_count) {
    	discard_current_block();
    }else {
        // Step events not completed yet...

        // Are we in acceleration phase ?
        if (step_events_completed <= accelerate_until) { // Calculate new timer value

          #if S_CURVE_ACCELERATION == 1
            // Get the next speed to use (Jerk limited!)
            uint32_t acc_step_rate = acceleration_time < current_block->acceleration_time
                                     ? eval_bezier_curve(acceleration_time)
                                     : current_block->cruise_rate;
          #else
            acc_step_rate = STEP_MULTIPLY(acceleration_time, current_block->acceleration_rate) + current_block->initial_rate;
            NOMORE(acc_step_rate, current_block->nominal_rate);
          #endif

            // acc_step_rate is in steps/second

            // step_rate to timer interval and steps per stepper isr
            interval = calc_timer_interval(acc_step_rate, &steps_per_isr);
            acceleration_time += interval;
        }
        // Are we in Deceleration phase ?
        else if (step_events_completed > decelerate_after) {
          uint32_t step_rate;

		  #if S_CURVE_ACCELERATION == 1
          // If this is the 1st time we process the 2nd half of the trapezoid...
          if (!bezier_2nd_half) {
            // Initialize the Bézier speed curve
            calc_bezier_curve_coeffs(current_block->cruise_rate, current_block->final_rate, current_block->deceleration_time_inverse);
            bezier_2nd_half = true;
            // The first point starts at cruise rate. Just save evaluation of the Bézier curve
            step_rate = current_block->cruise_rate;
          }
          else {
            // Calculate the next speed to use
            step_rate = deceleration_time < current_block->deceleration_time
              ? eval_bezier_curve(deceleration_time)
              : current_block->final_rate;
          }
        #else

          // Using the old trapezoidal control
          step_rate = STEP_MULTIPLY(deceleration_time, current_block->acceleration_rate);
          if (step_rate < acc_step_rate) { // Still decelerating?
            step_rate = acc_step_rate - step_rate;
            NOLESS(step_rate, current_block->final_rate);
          }
          else
            step_rate = current_block->final_rate;
        #endif

          // step_rate is in steps/second

          // step_rate to timer interval and steps per stepper isr
          interval = calc_timer_interval(step_rate, &steps_per_isr);
          deceleration_time += interval;

        }
        // Must be in cruise phase otherwise
	    else {

  		  // Calculate the ticks_nominal for this nominal speed, if not done yet
		  if (ticks_nominal < 0) {
		    // step_rate to timer interval and loops for the nominal speed
		    ticks_nominal = calc_timer_interval(current_block->nominal_rate, &steps_per_isr);
		  }

		  // The timer interval is just the nominal value for the nominal speed
		  interval = ticks_nominal;
	    }
    }
  }

  // Return the interval to wait
  return interval;
}

void Stepper_ISR()
{
	static uint32_t nextMainISR = 0;  // Interval until the next main Stepper Pulse phase (0 = Now)

	DISABLE_ISRS();

	// Count of ticks for the next ISR
	uint32_t next_isr_ticks = 0;

	do {
	   // Enable ISRs to reduce USART processing latency
	    ENABLE_ISRS();

	    if (!nextMainISR) pulse_phase_isr();                // 0 = Do coordinated axes Stepper pulses

	    if (!nextMainISR) nextMainISR = block_phase_isr();  // Manage acc/deceleration, get next block

	    // Get the interval to the next ISR call
	    const uint32_t interval = MIN_2(
	          nextMainISR                                       // Time until the next Pulse / Block phase
	          , (uint32_t)(HAL_TIMER_TYPE_MAX)                    // Come back in a very long time
	        );

	    //
		// Compute remaining time for each ISR phase
		//     NEVER : The phase is idle
		//      Zero : The phase will occur on the next ISR call
		//  Non-zero : The phase will occur on a future ISR call
		//

		nextMainISR -= interval;

		/**
		 * This needs to avoid a race-condition caused by interleaving
		 * of interrupts required by both the LA and Stepper algorithms.
		 *
		 * Assume the following tick times for stepper pulses:
		 *   Stepper ISR (S):  1 1000 2000 3000 4000
		 *   Linear Adv. (E): 10 1010 2010 3010 4010
		 *
		 * The current algorithm tries to interleave them, giving:
		 *  1:S 10:E 1000:S 1010:E 2000:S 2010:E 3000:S 3010:E 4000:S 4010:E
		 *
		 * Ideal timing would yield these delta periods:
		 *  1:S  9:E  990:S   10:E  990:S   10:E  990:S   10:E  990:S   10:E
		 *
		 * But, since each event must fire an ISR with a minimum duration, the
		 * minimum delta might be 900, so deltas under 900 get rounded up:
		 *  900:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E
		 *
		 * It works, but divides the speed of all motors by half, leading to a sudden
		 * reduction to 1/2 speed! Such jumps in speed lead to lost steps (not even
		 * accounting for double/quad stepping, which makes it even worse).
		 */

		// Compute the tick count for the next ISR
		next_isr_ticks += interval;

		/**
		 * The following section must be done with global interrupts disabled.
		 * We want nothing to interrupt it, as that could mess the calculations
		 * we do for the next value to program in the period register of the
		 * stepper timer and lead to skipped ISRs (if the value we happen to program
		 * is less than the current count due to something preempting between the
		 * read and the write of the new period value).
		 */
		DISABLE_ISRS();

	}while(0);

	// Set the next speed
	uint32_t period = next_isr_ticks - 1;

	htim1.Instance->ARR = period;
	htim1.Instance->CCR1 = period / 2;  // pulse width

	ENABLE_ISRS();
}

uint32_t Stepper_GetRealSpeed()
{
	uint32_t arr = htim1.Instance->ARR;
	if (arr == 0)
		return 0;

	if (arr >= 20000)
		return MINIMAL_STEPVELOCITY;

	uint32_t speed = (uint32_t)(STEPPER_TIMER_RATE) / arr;
	return speed;
}

void Stepper_Interpolation()
{
	g_InterpolationBlock.completsteps ++;

	UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	if(g_InterpolationBlock.time < g_InterpolationBlock.duration) {

		float vel = INP_calc_velocity(&g_InterpolationBlock, g_InterpolationBlock.time);

		if (vel < MINIMAL_STEPVELOCITY && vel >= 0.0){
			vel = MINIMAL_STEPVELOCITY;
		}else if (vel > -MINIMAL_STEPVELOCITY && vel < 0.0) {
			vel = -MINIMAL_STEPVELOCITY;
		}
        /* Save the last velocity*/
		//g_InterpolationBlock.lastvel = vel;

		if (g_InterpolationBlock.vel_to_cw){
			if (vel < 0.0){
				Stepper_SetDir(STEPPER_Dir_CW);
			}else{
				Stepper_SetDir(STEPPER_Dir_CCW);
			}
		}else{
			if (vel < 0.0){
				Stepper_SetDir(STEPPER_Dir_CCW);
			}else{
				Stepper_SetDir(STEPPER_Dir_CW);
			}
		}

		float absvel = fabsf(vel);
		g_InterpolationBlock.time += 1.0 / absvel;
		uint32_t interval = (uint32_t)(STEPPER_TIMER_RATE) / absvel;

		htim1.Instance->ARR = interval;
		htim1.Instance->CCR1 = interval / 2;
	}
	taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

	if (g_InterpolationBlock.completsteps >= g_InterpolationBlock.steps) {
		discard_interpolation_block();
	}
}

void Stepper_StepsISR()
{
	// record the count
	if (g_StepperData.dir == STEPPER_Dir_CW){
		g_StepperData.steps --;
	}else{
		g_StepperData.steps ++;
	}

	if (g_StepperData.steps == 0){
		Stepper_Stop();
	}
}

void Stepper_AngleISR()
{
	DC_UpdateAnglePID();
}

/*
 * User defined cb, Stepper Tim channel:
 **/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1) {
		// check the channel
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){

			if (g_StepperData.mode == RUN_Mode_StepsControl){
				Stepper_StepsISR();
				return;
			}

			if (g_StepperData.mode == RUN_Mode_AngleControl){
				Stepper_AngleISR();
				return;
			}

			if (g_InterpolationBlock.enable == 1) {
				Stepper_Interpolation();
			}else{
				if (current_block) {
					Stepper_ISR();
				}
			}
		}
	}
}
