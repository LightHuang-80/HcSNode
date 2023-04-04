/*
 * dspp.c
 *
 *  Created on: 2020��5��27��
 *      Author: Administrator
 */
#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "log.h"

#include "c402.h"
#include "c402def.h"

#include "PosProfile.h"
#include "PIDProfile.h"
#include "motiondef.h"
#include "motor.h"

#include "dspp.h"

extern MotionCtrlDef_t g_MotionCtrl;
extern int32_t         g_IncSteps;

typedef struct DS402_PP_Mode_ {
	DS402_PP_Status_t status;
	QueueHandle_t     taskQueue;
	MT_RequestBlock   reqblock;
	MHold_state_t     holdState;
}DS402_PP_Mode_t;

DS402_PP_Mode_t ProfPosMode;

void PP_reset()
{
	ProfPosMode.status = DS402_PP_Idle;
	ProfPosMode.holdState = MHOLD_Close;
	xQueueReset(ProfPosMode.taskQueue);
}

void PP_init()
{
	ProfPosMode.taskQueue = xQueueCreate(16, sizeof(MotionMsgItem_t));
	PP_reset();
}

void PP_active()
{
	/* Reset task queue*/
	taskENTER_CRITICAL();
	xQueueReset(ProfPosMode.taskQueue);
	taskEXIT_CRITICAL();

	ProfPosMode.status = DS402_PP_Idle;
	ProfPosMode.holdState = MHOLD_Close;
}

void PP_halt()
{
	taskENTER_CRITICAL();
	xQueueReset(ProfPosMode.taskQueue);

	MotionMsgItem_t item;

	item.code     = MMC_MotionHalt;
	item.position = 0;
	item.velocity = 0;
	item.acceleration = g_MotionCtrl.dece;

	xQueueSend(ProfPosMode.taskQueue, &item, 0);

	taskEXIT_CRITICAL();

	ProfPosMode.status = DS402_PP_Halt;
}

void PP_updateTask(uint8_t absOrRel)
{
	taskENTER_CRITICAL();

	xQueueReset(ProfPosMode.taskQueue);

	MotionMsgItem_t item;

	item.position = g_MotionCtrl.targetPos;
	item.velocity = g_MotionCtrl.profileVel;
	item.acceleration = g_MotionCtrl.acce;
	item.abs  = absOrRel;

	xQueueSend(ProfPosMode.taskQueue, &item, 0);

	taskEXIT_CRITICAL();
}

void PP_addNewTask(uint8_t absOrRel)
{
	taskENTER_CRITICAL();
	if (uxQueueGetQueueNumber(ProfPosMode.taskQueue) >= 16){
		/*Set pp err*/
		return;
	}

	MotionMsgItem_t item;

	item.position = g_MotionCtrl.targetPos;
	item.velocity = g_MotionCtrl.profileVel;
	item.acceleration = g_MotionCtrl.acce;
	item.abs  = absOrRel;

	xQueueSend(ProfPosMode.taskQueue, &item, 0);

	taskEXIT_CRITICAL();
}

uint16_t PP_exec(uint16_t controlWord, uint8_t newSetPoint)
{
	uint16_t ret = 0;

	uint8_t changeSetImmediately = 0;
	uint8_t abs  = 0;

	/* Halt bit 8*/
	if (controlWord & 0x100){
		/* Build a halt task*/
		/* motion halt with deceleration*/
		PP_halt();

		/* Reset the ack flag, bit 12*/
		g_MotionCtrl.statusWord &= ~0x1000;

		/* Reset the reach flag, bit 10*/
		g_MotionCtrl.statusWord &= ~0x400;
	}else{
		/* New set position bit 4*/
		if (!newSetPoint){
			/* Reset the ack flag, bit 12*/
			g_MotionCtrl.statusWord &= ~0x1000;
		}else{

			/* Change set immediately bit 5*/
			if (controlWord & 0x20){
				/*Set the motion posqueue pop position immediately*/
				changeSetImmediately = 1;
			}

			/* Absolution or relative position*/
			if (!(controlWord & 0x40)){
				abs = 1;
			}

			if (changeSetImmediately){
				/* Replace the current motion */
				PP_updateTask(abs);
			}else{
				/* Insert an new task*/
				PP_addNewTask(abs);
			}
		}
	}

	return ret;
}

boolean_T PP_isTaskTimeover(MT_RequestBlock* req)
{
	return PosProfile_StepOver(req->inhibittime);
}

void PP_reach(MT_RequestBlock* req)
{
	MOTOR_stop();

	if (ProfPosMode.status != DS402_PP_Halt){
		/* Set the status word bit 10*/
		g_MotionCtrl.statusWord |= 0x400;
	}

	ProfPosMode.status = DS402_PP_Idle;
	ProfPosMode.holdState = MHOLD_Close;

	req->state = MRS_Empty;
}

boolean_T PP_isReach(MT_RequestBlock* req)
{
	if (ProfPosMode.status == DS402_PP_Halt){
		return true;
	}

	uint32_t diff = abs(MOTOR_getPosDiff());
	if (diff <= CO_C402_Params.position_window){
		if (++(req->reachcounter) > CO_C402_Params.position_window_time)
			return true;
	}else{
		/*Reset reach counter*/
		req->reachcounter = 0;
	}

	return false;
}


void PP_startTask(MotionMsgItem_t* pitem, uint32_t ticks)
{
	real32_T ts = (real32_T)ticks / 1000.0f;

	/*Current velocity from motor model*/
	real32_T curVel = (real32_T)MOTOR_getSpeed();

	uint32_t maxJerk = pitem->acceleration;

	if (pitem->code == MMC_MotionHalt){
		/*Act halt definition, use halt deceleration
		 * start halt motion*/
		PosProfile_StartHalt(curVel/200.0f, maxJerk/200.0f, ts);

		pitem->position = PosProfile_GetTargetPos();
		MOTOR_setRelTarget(pitem->position);

		ProfPosMode.status = DS402_PP_Halt;
	}else{
		/*0x6083  accelerate*/
		real32_T maxAcce = pitem->acceleration / 200.0F;

		/*0x6081  profile velocity*/
		real32_T maxVel  = pitem->velocity / 200.0F;

		if (pitem->abs)
			MOTOR_setAbsTarget(pitem->position);
		else
			MOTOR_setRelTarget(pitem->position);

		pitem->position = MOTOR_getPosDiff();

		/*Motion start from current pos to request position*/
		PosProfile_Start(pitem->position/200.0f,
						 curVel/200.0f, 0,
						 maxVel, maxAcce, maxJerk/200.0F, ts);

		/*Acknowledge a motion request*/
		/* New set point acknowledged
		* Set status word bit 12*/
		g_MotionCtrl.statusWord |= 0x1000;

		/* Reset the status word reach target flag, bit10*/
		g_MotionCtrl.statusWord &= ~0x400;

		ProfPosMode.status = DS402_PP_Running;
	}

	/*Setting the current motion request, start inhibit timer*/
	ProfPosMode.reqblock.state = MRS_Running;
	ProfPosMode.reqblock.inhibittime = 0;
	ProfPosMode.reqblock.reachcounter = 0;

	/*Stop the PID correcting*/
	ProfPosMode.holdState = MHOLD_Close;

	/*Start motor*/
	MOTOR_run();
}


void PP_step(MT_RequestBlock* req, uint32_t ticks)
{
	int32_t perr = MOTOR_getPosDiff();

	PosProfile_Step(perr);
	MOTOR_setSpeed((int32_t)roundf(rtY.CmdVel * 200.0f));

	req->inhibittime += ticks;
}


void PP_holdStep(uint32_t ticks)
{
	real32_T ts = ticks/1000.0f;

	PIDProfile_Step(ts, g_IncSteps);
	MOTOR_setSpeed((int32_t)(rtPIDY.CmdVel));
}

void PP_hold(MT_RequestBlock* req, uint32_t ticks)
{
	uint32_t diff = abs(MOTOR_getPosDiff());
	if (diff > CO_C402_Params.position_window){
		if (ProfPosMode.holdState == MHOLD_Close){

			/*Correcting PID start, from current pos to target position*/
			PIDProfile_Start(MOTOR_getTarget(), 1200.0f, 1000.0f, 1000.0f);
			ProfPosMode.holdState = MHOLD_Open;
			MOTOR_run();
		}
	}

	if (ProfPosMode.holdState == MHOLD_Open){
		PP_holdStep(ticks);
	}
}

void PP_process(uint32_t ticks)
{
	MotionMsgItem_t item;

	if (ProfPosMode.status == DS402_PP_Idle ||
			g_MotionCtrl.controlWord & 0x100 /*Halt bit set*/ ||
			g_MotionCtrl.controlWord & 0x20  /*Change on immediate bit set*/){

		/*High priority motion*/
		BaseType_t xTaskWokenByReceive = pdFALSE;

		UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
		BaseType_t result = xQueueReceiveFromISR(ProfPosMode.taskQueue, &item, &xTaskWokenByReceive);
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

		if (result == pdPASS) {
			/*Motion plan and run*/
			PP_startTask(&item, ticks);
		}

		if(xTaskWokenByReceive == pdTRUE) {
			portYIELD_FROM_ISR (xTaskWokenByReceive);
		}
	}

	/*step*/
	if (ProfPosMode.status != DS402_PP_Idle){
		if (ProfPosMode.reqblock.state != MRS_Empty){
			if (!PP_isTaskTimeover(&ProfPosMode.reqblock)){
				PP_step(&ProfPosMode.reqblock, ticks);
				return;
			}else{
				if (PP_isReach(&ProfPosMode.reqblock)){
					PP_reach(&ProfPosMode.reqblock);
					return;
				}
			}
		}
	}

	/*prepare a correcting phase, and correcting step*/
	PP_hold(&ProfPosMode.reqblock, ticks);
}
