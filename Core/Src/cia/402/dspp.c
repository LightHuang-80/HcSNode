/*
 * dspp.c
 *
 *  Created on: 2020��5��27��
 *      Author: Administrator
 */
#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "c402.h"
#include "c402def.h"
#include "dspp.h"

#include "nmotion.h"
#include "log.h"

extern MotionCtrlDef_t g_MotionCtrl;
extern Node_DriveProfile_t g_NodeDriveProfile;

typedef void (*pTargetReachedCallback)(int32_t);

typedef struct DS402_PP_Mode_ {
	DS402_PP_Status_t status;
	int8_t            setPosBit;
}DS402_PP_Mode_t;

DS402_PP_Mode_t g_ProfPosMode;

void PP_reset()
{
	g_ProfPosMode.status = DS402_PP_Idle;
	g_ProfPosMode.setPosBit = 0;
}

void PP_Init()
{
	PP_reset();
}

void PP_OnTargetReach(int32_t position)
{
	/* Reset the bit4*/
	g_ProfPosMode.setPosBit = 0;

	/* Set the status word bit 10*/
	g_MotionCtrl.statusWord |= 0x400;

	g_ProfPosMode.status = DS402_PP_Idle;
}

void PP_OnMotionStart()
{
	/* New set point acknowledged
	 * Set status word bit 12*/
	g_MotionCtrl.statusWord |= 0x1000;

	/* Reset the status word reach target flag, bit10*/
	g_MotionCtrl.statusWord &= ~0x400;

	g_ProfPosMode.status = DS402_PP_Running;
}

void PP_active(uint16_t controlWord)
{
	/* Reset the bit4*/
	g_ProfPosMode.setPosBit = 0;

	MT_setMotionStartPreCallback(PP_OnMotionStart);
	MT_setReachCallback(PP_OnTargetReach);
}

uint16_t PP_exec(uint16_t controlWord)
{
	uint16_t ret = 0;

	uint8_t newSetPoint = 0;
	uint8_t changeSetImmediately = 0;
	uint8_t halt = 0;
	uint8_t abs  = 0;

	/* Halt bit 8*/
	if (controlWord & 0x100){
		g_ProfPosMode.status = DS402_PP_Halt;

		/*Reset the motion*/
		halt = 1;
	}else{
		/* New set position bit 4*/
		if (controlWord & 0x10){
			if (!g_ProfPosMode.setPosBit){
				/* Start an new position motion task*/
				newSetPoint = 1;
			}
			g_ProfPosMode.setPosBit = 1;
		}else{
			g_ProfPosMode.setPosBit = 0;

			/* Reset the ack flag, bit 12*/
			g_MotionCtrl.statusWord &= ~0x1000;
		}

		/* Change set immediately bit 5*/
		if (controlWord & 0x20){
			/*Set the motion posqueue pop position immediately*/
			changeSetImmediately = 1;
		}

		/* Absolution or relative position*/
		if (!(controlWord & 0x40)){
			abs = 1;
		}
	}

	/* Build motion task
	 * Peak velocity 0x6081
	 * Acceleration/deceleration 0x6083
	 * */
	if (halt){
		/* Build a halt task*/
		/* motion halt with deceleration*/
		MT_Halt(g_MotionCtrl.dece);
	}else{
		if (newSetPoint){
			if (changeSetImmediately){
				/* Replace the current motion */
				MT_UpdateTask(abs, g_MotionCtrl.targetPos, g_MotionCtrl.profileVel, g_MotionCtrl.acce);
			}else{
				/* Insert an new task*/
				MT_NewTask(abs, g_MotionCtrl.targetPos, g_MotionCtrl.profileVel, g_MotionCtrl.acce);
			}
		}
	}

	return ret;
}
