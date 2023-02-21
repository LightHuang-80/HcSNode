/*
 * interpolation.h
 *
 *  Created on: Dec 1, 2020
 *      Author: Administrator
 */

#ifndef SRC_MOTION_INTERPOLATION_H_
#define SRC_MOTION_INTERPOLATION_H_

typedef struct InterpolationBlock_ {

	float duration;
	float dist;

	float lastvel;
	float lastacce;

	float a0;
	float a1;
	float a2;
	float a3;
	float a4;
	float a5;

	float time;

	uint32_t steps;
	uint32_t completsteps;
	uint32_t vel_interval;
	int8_t   vel_to_cw;
	uint8_t  enable;

}InterpolationBlock_t;

extern InterpolationBlock_t g_InterpolationBlock;

void INP_calc_coeffs(InterpolationBlock_t* block, int32_t curpos, int32_t targetpos,
		float targetvel, float targetacce, float duration);
float INP_calc_velocity(InterpolationBlock_t *block, float tick);
void INP_timer_update();
void INP_Prepare();
#endif /* SRC_MOTION_INTERPOLATION_H_ */
