/*
 * nstepper.c
 *
 *  Created on: 2023年3月6日
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

#include "nstepper.h"

#if !defined (Stepper_Dir_Pin)
  #define Stepper_Dir_Pin GPIO_PIN_9
  #define Stepper_Dir_GPIO_Port GPIOC
#endif

#define HAL_TIMER_TYPE_MAX 0xFFFF

#define RUN_Mode_Normal        0x01
#define RUN_Mode_StepsControl  0x02
#define RUN_Mode_AngleControl  0x03

typedef void (*Stepper_Motion_Finished_CB)(int8_t type);

void Stepper_ISR();

extern uint16_t g_AbsAngle;

int32_t delta_error;
uint32_t acceleration_time, deceleration_time;
uint8_t  steps_per_isr;
bool     abort_current_block;
uint32_t step_events_completed = 0, // The number of step events executed in the current block
         accelerate_until,          // The count at which to stop accelerating
         decelerate_after,          // The count at which to start decelerating
         step_event_count;          // The total event count for the current block
uint8_t  last_direction_bits;
block_t* current_block;
int32_t  ticks_nominal = -1;
uint8_t  oversampling_factor = 0;

typedef struct STEPPER_RUN_DATA{
	int32_t targetPos;
	int32_t currentPos;

	int32_t steps;      // steps

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

/*完成当前block*/
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

/*
 * Stepper motor Initialize
 * microsteps
 */
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

/*
 *  Flip stepper motor direction
 */
void Stepper_ChangeDir()
{
	if (g_StepperData.dir == STEPPER_Dir_CW)
		g_StepperData.dir = STEPPER_Dir_CCW;
	else
		g_StepperData.dir = STEPPER_Dir_CW;

	HAL_GPIO_TogglePin(Stepper_Dir_GPIO_Port, Stepper_Dir_Pin);
}

/*
 *  Get current stepper motor direction
 */
int8_t Stepper_GetDir()
{
	return g_StepperData.dir;
}

/*
 *  Set stepper motor direction
 */
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

/*
 *  Set stepper motor speed
 */
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

/*
 *  Get stepper motor current speed
 */
uint16_t Stepper_GetSpeed()
{
	return g_StepperData.speed;
}

/* Stepper motor go to origin position*/
void Stepper_GoHome()
{
	int32_t steps = 0 - g_StepperData.currentPos;
	Stepper_RunSteps(steps);
}

/* Stepper motor stop*/
void Stepper_Stop()
{
	g_StepperData.state = STEPPER_State_Stop;
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);

	//Clear all of blocks
	discard_current_block();
}

/* Get stepper motor current state*/
uint16_t Stepper_GetState()
{
	return g_StepperData.state;
}

/* Stepper motor run*/
void Stepper_Run()
{
	g_StepperData.mode  = RUN_Mode_Normal;
	g_StepperData.state = STEPPER_State_Running;
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
}

/* Stepper motor run with motion profile setting */
void Stepper_RunWithSpeed(int16_t speed)
{
	if (speed >= 0){
		Stepper_SetDir(STEPPER_Dir_CW);
	}else{
		Stepper_SetDir(STEPPER_Dir_CCW);
	}

	Stepper_SetSpeed(abs(speed));
	g_StepperData.mode  = RUN_Mode_Normal;
	g_StepperData.state = STEPPER_State_Running;
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
}

/* Stepper motor go steps */
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


/* Per step interrupt service routine*/
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

/*
 * User defined cb, Stepper Tim channel:
 **/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1) {
		// check the channel
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){

			/*Per step call back stepsisr*/
			if (g_StepperData.mode == RUN_Mode_StepsControl){
				Stepper_StepsISR();
				return;
			}

			/**/
		}
	}
}
