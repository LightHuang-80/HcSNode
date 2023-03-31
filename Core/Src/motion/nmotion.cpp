/*
 * nmotion.c
 *
 *  Created on: 2023年3月6日
 *      Author: Administrator
 */


/* Motion 本质上处理motor运动问题，运动包括：
 * 1. 设置位置
 * 2. 设置速度,加速度及减速度
 * 3. 设置方向
 * 4. 启动及停止, 电机使能和释放
 * 5. 位置零点处理
 * 5. 监视当前位置,并进行修正
 * 6. 模式管理
* 对于给定的Position，以多种方式到达指定位置，并在到达后，发出reach信号。
 * 是对Motor的高级封装
 *  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "list.h"

#include "main.h"
#include "tim.h"
#include "c402.h"
#include "nstepper.h"
#include "log.h"
#include "planner.h"

#include "nmotion.h"
#include "mcdrv.h"

#include "motor.h"

extern volatile int32_t    g_IncSteps;
extern Node_DriveProfile_t g_NodeDriveProfile;
extern MotionCtrlDef_t     g_MotionCtrl;

#define MOTION_INVAILD_POS	0x7FFFFFFF
#define MIN_DIFF_DISTANCE   5

#define SCURVE_SPEEDTHRESHOLD  300
#define MOTION_IDLESAMPLETIME  300

MT_MotionDrive_t g_MotionDriveHandle;

typedef struct MT_ResultBlock_ {
	int32_t  request_len;    // steps
	int32_t  missed_len;     // steps
	uint32_t estimate_time;  // ms
	uint32_t use_time;       // ms
}MT_ResultBlock;

void MT_Clear()
{
	g_MotionDriveHandle.target = g_IncSteps;

	g_MotionDriveHandle.switchPoint = 0;
	g_MotionDriveHandle.homePoint = 0;

	g_MotionDriveHandle.adMode = MT_ADMode_Normal;
	g_MotionDriveHandle.lock   = 0;

	/*Default settings*/
	g_MotionDriveHandle.reachWindow = 8;

	g_MotionDriveHandle.state = MTS_Idle;
	g_MotionDriveHandle.reqblock.state = MRS_Empty;

	g_MotionDriveHandle.reachCB = NULL;
	g_MotionDriveHandle.motionStartPreCB = NULL;

	g_MotionDriveHandle.correctingState  = MCRT_Stop;

	/*Initialize the profile parameters*/
	MT_setProfile(&g_NodeDriveProfile);
}

void MT_Reset()
{
	MT_Clear();

	/*Reset the queue and list*/
	if (g_MotionDriveHandle.msgQueue){
		xQueueReset( g_MotionDriveHandle.msgQueue);
	}
}

void MT_Begin()
{
	MT_Reset();

	/*Start motor PWM timer*/
	HAL_TIM_Base_Start_IT(&htim5);
}

void MT_Loop(uint32_t ticks)
{
	/*Synchronize the current steps*/
	g_MotionCtrl.currentPos = g_IncSteps;

	/*Synchronize the current velocity*/
	g_MotionCtrl.currentVel = MOTOR_getSpeed();

	/*Check motor status routine*/
	MOTOR_loop(ticks);
}

void MT_setProfile(Node_DriveProfile_t* profile)
{
	MOTOR_setPolarity(-1);
	MOTOR_setAbsTarget(g_IncSteps);
	g_MotionDriveHandle.switchdir = profile->switchdir; // default 0
}

void MT_Init(QueueHandle_t msgQueue)
{
	MT_Reset();

	g_MotionDriveHandle.msgQueue = msgQueue;
	g_MotionDriveHandle.reqblock.state = MRS_Empty;

	/* Initialize position profile model*/
	PosProfile_Initialize();

	/* Initialize pid profile*/
	PIDProfile_Initialize(0.49f, 1.21f, -0.28f, 1.76f);
}

void MT_setReachCallback(void (*pFunct)(int32_t position))
{
	g_MotionDriveHandle.reachCB = pFunct;
}

void MT_setMotionStartPreCallback(void (*pFunct)())
{
	g_MotionDriveHandle.motionStartPreCB = pFunct;
}

/*32 multiples number for window is the best*/
void MT_setReachWindow(int32_t window)
{
	vPortEnterCritical();
	g_MotionDriveHandle.reachWindow = window;
	vPortExitCritical();
}

void MT_Start(MotionMsgItem_t* pitem, uint32_t ticks)
{
	real32_T ts = (real32_T)ticks / 1000.0f;

	/*Current velocity from motor model*/
	real32_T curVel = (real32_T)MOTOR_getSpeed();

	uint32_t maxJerk = pitem->acceleration;

	if (pitem->code == MMC_MotionHalt){
		/*Act halt definition, use halt deceleration
		 * start halt motion*/
		PosProfile_StartHalt(curVel, maxJerk, ts);
		pitem->position = PosProfile_GetTargetPos();
		MOTOR_setRelTarget(pitem->position);
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
		if (g_MotionDriveHandle.motionStartPreCB){
			g_MotionDriveHandle.motionStartPreCB();
		}
	}

	/*Setting the current motion request, start inhibit timer*/
	g_MotionDriveHandle.reqblock.state = MRS_Running;
	g_MotionDriveHandle.reqblock.inhibittime = 0;
	g_MotionDriveHandle.reqblock.reachcounter = 0;

	/*Stop the PID correcting*/
	g_MotionDriveHandle.correctingState = MCRT_Stop;

	/*Reset the motion state*/
	if (pitem->code == MMC_TargetSet)
		g_MotionDriveHandle.state = MTS_Running;
	else if (pitem->code == MMC_MotionUpdated)
		g_MotionDriveHandle.state = MTS_Updated;

	/*Start motor*/
	MOTOR_run();
}

void MT_ProfileStep(MT_RequestBlock* req, uint32_t ticks)
{
	int32_t perr = MOTOR_getPosDiff();

	PosProfile_Step(perr);
	MOTOR_setSpeed((int32_t)roundf(rtY.CmdVel * 200.0f));

	req->inhibittime += ticks;
}

boolean_T MT_OverTime(MT_RequestBlock* req)
{
	return PosProfile_StepOver(req->inhibittime);
}

void MT_Reach(MT_RequestBlock* req)
{
	MOTOR_stop();

	if (g_MotionDriveHandle.state != MTS_Halting){
		if (g_MotionDriveHandle.reachCB){
			g_MotionDriveHandle.reachCB(g_IncSteps);
		}
	}

	g_MotionDriveHandle.state = MTS_Idle;
	req->state = MRS_Empty;
}

boolean_T MT_IsReach(MT_RequestBlock* req)
{
	if (g_MotionDriveHandle.state == MTS_Halting){
		/*do motor stop*/
		return true;
	}

	int32_t diff = abs(MOTOR_getPosDiff());
	if (diff <= g_MotionDriveHandle.reachWindow){
		if (++(req->reachcounter) > REACH_OVER_COUNT)
			return true;
	}else{
		/*Reset reach counter*/
		req->reachcounter = 0;
	}

	return false;
}

void MT_CorrectingStep(uint32_t ticks)
{
	real32_T ts = ticks/1000.0f;

	PIDProfile_Step(ts, g_IncSteps);
	MOTOR_setSpeed((int32_t)(rtPIDY.CmdVel));
}

void MT_Correcting(MT_RequestBlock* req, uint32_t ticks)
{
	int32_t diff = abs(MOTOR_getPosDiff());
	if (diff > g_MotionDriveHandle.reachWindow){
		if (g_MotionDriveHandle.correctingState == MCRT_Stop){

			/*Correcting PID start, from current pos to target position*/
			PIDProfile_Start(MOTOR_getTarget(), 1200.0f, 1000.0f, 1000.0f);
			g_MotionDriveHandle.correctingState = MCRT_Act;
			MOTOR_run();
		}
	}

	if (g_MotionDriveHandle.correctingState == MCRT_Act){
		MT_CorrectingStep(ticks);
	}
}

void MT_Halt(uint32_t dece)
{
	/* Use the deceleration defined in 0x6084 to halt*/
	g_MotionDriveHandle.state = MTS_Halting;

	/* Clear motion item queue*/
	if (uxQueueGetQueueNumber(g_MotionDriveHandle.msgQueue) > 0){
		xQueueReset(g_MotionDriveHandle.msgQueue);
	}

	MotionMsgItem_t item;

	item.position = 0;
	item.velocity = 0;
	item.acceleration = dece;
	item.code = MMC_MotionHalt;

	xQueueSend(g_MotionDriveHandle.msgQueue, &item, 0);
}

void MT_UpdateTask(uint8_t absolute, int32_t target, int32_t vel, uint32_t acce)
{
	/* Update current motion task*/
	g_MotionDriveHandle.state = MTS_Updated;

	/* Clear motion item queue*/
	if (uxQueueGetQueueNumber(g_MotionDriveHandle.msgQueue) > 0){
		xQueueReset(g_MotionDriveHandle.msgQueue);
	}

	MotionMsgItem_t item;

	item.position = target;
	item.velocity = vel;
	item.acceleration = acce;
	item.code = MMC_MotionUpdated;
	item.abs  = absolute;

	xQueueSend(g_MotionDriveHandle.msgQueue, &item, 0);
}

void MT_NewTask(uint8_t absolute, int32_t target, int32_t vel, uint32_t acce)
{
	/* Create new motion task and insert into msg queue*/
	MotionMsgItem_t item;

	item.position = target;
	item.velocity = vel;
	item.acceleration = acce;
	item.code = MMC_TargetSet;
	item.abs  = absolute;
	xQueueSend(g_MotionDriveHandle.msgQueue, &item, 0);
}

void MT_process_v3(uint32_t ticks)
{
	MotionMsgItem_t item;

	if (g_MotionDriveHandle.state == MTS_Idle ||
		g_MotionDriveHandle.state == MTS_Halting ||
		g_MotionDriveHandle.state == MTS_Updated){
		/*High priority motion*/
		BaseType_t xTaskWokenByReceive = pdFALSE;
		BaseType_t result = xQueueReceiveFromISR(g_MotionDriveHandle.msgQueue, &item, &xTaskWokenByReceive);
		if (result == pdPASS) {
			/*Motion plan and run*/
			MT_Start(&item, ticks);
		}

		if(xTaskWokenByReceive == pdTRUE) {
			portYIELD_FROM_ISR (xTaskWokenByReceive);
		}
	}

	/*step*/
	if (g_MotionDriveHandle.state == MTS_Halting ||
		g_MotionDriveHandle.state == MTS_Updated ||
		g_MotionDriveHandle.state == MTS_Running){

		if (g_MotionDriveHandle.reqblock.state != MRS_Empty){
			if (!MT_OverTime(&g_MotionDriveHandle.reqblock)){
				MT_ProfileStep(&g_MotionDriveHandle.reqblock, ticks);
				return;
			}else{
				if (MT_IsReach(&g_MotionDriveHandle.reqblock)){
					MT_Reach(&g_MotionDriveHandle.reqblock);
					return;
				}
			}
		}
	}

	/*prepare a correcting phase, and correcting step*/
	MT_Correcting(&g_MotionDriveHandle.reqblock, ticks);
}

