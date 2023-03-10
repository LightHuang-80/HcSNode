/*
 * dspp.c
 *
 *  Created on: 2020��5��27��
 *      Author: Administrator
 */
#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "c402def.h"
#include "dspp.h"

#include "nmotion.h"
#include "log.h"

typedef void (*pTargetReachedCallback)(int32_t);

typedef struct DS402_PP_Data_ {

	uint32_t speed;
	uint32_t reachWindow;
	DS402_PP_Status_t status;
	pTargetReachedCallback targetReachedCB;

}DS402_PP_Data_t;

DS402_PP_Data_t g_MotorPPData;

void PP_reset()
{
	g_MotorPPData.reachWindow = 32;
	g_MotorPPData.status = DS402_PP_None;
}

void PP_setSpeed(uint32_t speed)
{
	g_MotorPPData.speed = speed;
}

void PP_Init(uint32_t speed)
{
	PP_setSpeed(speed);
	g_MotorPPData.targetReachedCB = NULL;

	PP_reset();
}

void PP_setTargetReachedCallback(void (*pfunct)(int32_t position))
{
	g_MotorPPData.targetReachedCB = pfunct;
}

void PP_OnTargetReach(int32_t position)
{
	// LOG_Print(LOG_InfoLevel, "PP reach the position: %ld\n", position);
	if (g_MotorPPData.targetReachedCB){
		g_MotorPPData.targetReachedCB(position);
	}
}

void PP_active()
{
	if (g_MotorPPData.status == DS402_PP_None) {
		g_MotorPPData.status = DS402_PP_Running;

		MT_setReachCallback(PP_OnTargetReach);
		MT_setReachWindow(g_MotorPPData.reachWindow);
		MT_setSpeed(g_MotorPPData.speed);
	}
}

uint16_t PP_exec(uint16_t controlWord)
{
	uint16_t ret = 0;

	uint16_t newSetPoint = controlWord & 0x10;
	uint16_t changeSetImmediately = controlWord & 0x20;
	uint16_t halt = controlWord & 0x100;

	if (halt){
		/*Reset the motion*/
		return ret;
	}

	if (newSetPoint){
		/*Buffer the target*/
	}

	if (changeSetImmediately){
		/*Set the motion posqueue pop position immediately*/
	}

	return ret;
}
