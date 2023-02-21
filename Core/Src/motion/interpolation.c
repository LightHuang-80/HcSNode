/*
 * interpolation.c
 *
 *  Created on: Dec 1, 2020
 *      Author: Administrator
 */




#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
//#include "motion.h"
#include "planner.h"
#include "interpolation.h"
#include "stepper.h"

InterpolationBlock_t g_InterpolationBlock;

void INP_calc_coeffs(InterpolationBlock_t* block, int32_t curpos, int32_t targetpos,
		float targetvel, float targetacce, float duration)
{
	block->dist = targetpos - curpos;
	block->duration = duration;
	block->steps = (uint32_t)ceilf(fabsf(block->dist));
	block->completsteps = 0;

	block->a0 = curpos;
	block->a1 = block->lastvel;
	block->a2 = block->lastacce / 2.0f;

	/*
	 * a3 = (20*h - (8*v1 + 12*v0)*T - (3*acc0 - acc1)*T**2) / (2*T**3)
       a4 = (-30*h + (14*v1 + 16*v0)*T + (3*acc0 - 2*acc1)*T**2) / (2*T**4)
       a5 = (12*h - 6*(v1 + v0)*T + (acc1 - acc0)*T**2) / (2*T**5)
     */
	float tpow2 = block->duration * block->duration;
	float tpow3 = tpow2 * block->duration;
	float tpow4 = tpow3 * block->duration;
	float tpow5 = tpow4 * block->duration;

	block->a3 = (20.0f* block->dist - (8.0f*targetvel + 12.0f*block->lastvel)*block->duration -
			    (3.0f * block->lastacce - targetacce)*tpow2) / (2.0*tpow3);

	block->a4 = (-30.0f*block->dist + (14.0f*targetvel + 16.0*block->lastvel)*block->duration +
			    (3.0*block->lastacce - 2.0*targetacce)*tpow2) / (2.0*tpow4);

	block->a5 = (12.0*block->dist - 6.0*(targetvel + block->lastvel)*block->duration +
			    (targetacce - block->lastacce)*tpow2) / (2.0*tpow5);

	block->lastvel = targetvel;
	block->lastacce = targetacce;

	block->time = 0;
}

float INP_calc_velocity(InterpolationBlock_t *block, float tick)
{
	/*
	  a1 + 2*a2*(t - t0) + 3*a3*(t - t0)**2 + 4*a4*(t - t0)**3 + 5*a5*(t - t0)**4 # velocity
	*/

	float velocity;

	/*
	if (block->a2 == 0 && block->lastacce == 0) {
		// 匀速运动，防止在速度插值引起波动
		velocity = block->a1;
		return velocity;
	}*/

	float tpow2 = tick*tick;
	float tpow3 = tpow2*tick;
	float tpow4 = tpow3*tick;

	velocity =
			block->a1 + 2.0*tick*block->a2 + 3.0*tpow2*block->a3 + 4.0*tpow3*block->a4 + 5.0*tpow4*block->a5;
	return velocity;

}

void INP_timer_update()
{
	/*
	static int32_t step = 0;
	if (step >= 18){
		step = 0;
		HAL_TIM_Base_Stop_IT(&htim5);
		return;
	}

	float pos[18] = {0.862089, 0.867593, 0.882057,
			         0.906379, 0.940051, 0.981254,
					 1.02349, 1.065559,1.107729,
					 1.149897,1.192059,1.234289,
					 1.276415,1.316921,1.349684,
					 1.373368,1.387018,1.391966};
	float vel[18] = {0,       0.10591, 0.19626,
			         0.28704, 0.38304, 0.42167,
			         0.42167, 0.42167, 0.42167,
			         0.42167, 0.42167, 0.42167,
			         0.42167, 0.36876, 0.28057,
			         0.1955,  0.09546, 0};
    float acce[18] ={0.00629, 0.36631, 0.23137,
    		         0.93881, 0.84802, 0,
					 0,		0, 		0,
    		         0,		0,		0,
    		         0,  	-0.75024, -0.86727,
    		        -0.08709,-0.50329, 0};

    float ratio = 9356.590;

    MotionMsgItem_t item;
    item.position = pos[step] * ratio;
    item.velocity = vel[step] * ratio;
    item.acceleration = acce[step] * ratio;
    item.duration = 1.0;
    item.type = 1;
    item.code = MMC_TargetSet;
    BaseType_t highTaskWoken = pdFALSE;
	if (pdPASS == xQueueSendFromISR(g_MotionDriveData.posQueue, &item, &highTaskWoken)){
	}

	if(highTaskWoken == pdTRUE) {
		portYIELD_FROM_ISR (highTaskWoken);
	}

	step ++;
	*/
}

void INP_Prepare()
{
	g_InterpolationBlock.enable = 1;

	float v0 = fabsf(g_InterpolationBlock.a1);
	if (v0 <= MINIMAL_STEPVELOCITY) {
		/*规范最小速度，防止过小速度导致无法启动*/
		v0 = MINIMAL_STEPVELOCITY;
	}

	uint32_t interval = (uint32_t)((float)STEPPER_TIMER_RATE) / v0;
	htim1.Instance->ARR = interval;
	htim1.Instance->CCR3 = interval / 2;
}
