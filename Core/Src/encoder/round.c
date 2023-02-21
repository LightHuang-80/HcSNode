/*
 * round.c
 *
 *  Created on: 2020年8月20日
 *      Author: Administrator
 */

#include <stdlib.h>
#include <stdio.h>

#include "main.h"
#include "as5048a.h"

#define ANGLE_RESOLUTION 16384 // 14bit resolution
#define RES_QUADRANT     (ANGLE_RESOLUTION >> 2)

typedef struct ROUND_INC{
	uint16_t quat[5];

	uint16_t homeAngle;
	int16_t  RantInc;  // 象限增加计数器
	int16_t  quadRant; // 当前象限
}Round_Inc_t;

Round_Inc_t      g_RoundRecord;

void RND_Init(uint16_t angle)
{
	// Set the home angle
	g_RoundRecord.homeAngle = angle;

	g_RoundRecord.quat[0] = angle;
	g_RoundRecord.quat[1] = (g_RoundRecord.quat[0] + RES_QUADRANT) % (ANGLE_RESOLUTION);
	g_RoundRecord.quat[2] = (g_RoundRecord.quat[1] + RES_QUADRANT) % (ANGLE_RESOLUTION);
	g_RoundRecord.quat[3] = (g_RoundRecord.quat[2] + RES_QUADRANT) % (ANGLE_RESOLUTION);
	g_RoundRecord.quat[4] = (angle) % (ANGLE_RESOLUTION);

	// 4象限, homeAngle 设置为第0象限
	g_RoundRecord.quadRant = 0;

	// 初始象限增加值为0
	g_RoundRecord.RantInc = 0;
}

void RND_Input(uint16_t angle)
{
	// 当前在第几象限
	int16_t rant = -1;

	for (int16_t i = 0; i < 4; i++){
		if (angle >= g_RoundRecord.quat[i] && angle < g_RoundRecord.quat[i+1]){
			rant = i;
			break;
		}

		if (g_RoundRecord.quat[i] > g_RoundRecord.quat[i+1]){
			if ((angle >= g_RoundRecord.quat[i] && angle <= ANGLE_RESOLUTION) ||
				(angle >= 0 && angle < g_RoundRecord.quat[i+1])){
				rant = i;
				break;
			}
		}
	}

	if (abs(g_RoundRecord.quadRant - rant) == 2){
		// 转速太快, 失步, 或角度数据读取不出
		printf("Lost steps too much.\n");
		return;
	}

	if (rant == -1) {
		printf("Not an effective angle, %d, home: %d\n", angle, g_RoundRecord.homeAngle);
		return;
	}

	// 计算象限增加值
	if (rant == g_RoundRecord.quadRant - 1)
		g_RoundRecord.RantInc --;
	else if (rant == g_RoundRecord.quadRant + 1)
		g_RoundRecord.RantInc ++;
	else {
		if (g_RoundRecord.quadRant == 0 && rant == 3)
			g_RoundRecord.RantInc --;
		else if (g_RoundRecord.quadRant == 3 && rant == 0)
			g_RoundRecord.RantInc ++;
	}

	// 象限设置
	g_RoundRecord.quadRant = rant;
}

uint16_t RND_GetHomeAngle()
{
	return g_RoundRecord.homeAngle;
}

/*
 * 角度传感器，逆时针 角度增大
 * */
int32_t RND_GetCrossAngle(uint16_t angle)
{
	int32_t delta;
	int32_t side;

	int32_t home   = g_RoundRecord.homeAngle;
	int32_t homePi = (home + ANGLE_RESOLUTION / 2) % ANGLE_RESOLUTION;

	if (home > homePi) {
		// home 在0点右边
		if ((angle >= 0 && angle < homePi) || (angle >= home && angle <= ANGLE_RESOLUTION)) {
			// angle 在 home的左边
			delta = (angle + ANGLE_RESOLUTION - home) % ANGLE_RESOLUTION;
			side = 1;
		}else {
			// angle 在 home的左边
			delta = home - angle;
			side = -1;
		}
	}else {
		// home 在 0 点的左边
		if (angle >= home && angle < homePi) {
			// angle 在 home的左边
			delta = angle - home;
			side = 1;
		}else {
			// angle 在 home的右边
			delta = (home - angle + ANGLE_RESOLUTION) % ANGLE_RESOLUTION;
			side = -1;
		}
	}

	delta *= side;

	int32_t round;
	if (g_RoundRecord.RantInc >= 0){
		round = g_RoundRecord.RantInc / 4;
	}else{
		round = (g_RoundRecord.RantInc + 1) / 4;
	}

	if (g_RoundRecord.RantInc > 0) {
		if (delta >= 0) {
			delta += round * ANGLE_RESOLUTION;
		}else {
			delta += (round + 1)* ANGLE_RESOLUTION;
		}
	}else if (g_RoundRecord.RantInc < -1) {
		if (delta < 0){
			delta += round * ANGLE_RESOLUTION;
		}else {
			delta += (round - 1)* ANGLE_RESOLUTION;
		}
	}

	return delta;
}


int16_t RND_GetRound()
{
	int16_t round = 0;

	if (g_RoundRecord.RantInc >= 0){
		round = g_RoundRecord.RantInc / 4;
	}else{
		round = (g_RoundRecord.RantInc + 1) / 4;
	}

	return round;
}

int32_t RND_GetIncSteps(uint16_t angle, float ms)
{
	float ca = (float)RND_GetCrossAngle(angle);

	// 一圈 = 200 * 细分，
	int32_t steps = ca * (200.0 * ms) / (float)ANGLE_RESOLUTION;
	return steps;
}
