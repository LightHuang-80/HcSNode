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

extern volatile int32_t    g_IncSteps;
extern Node_DriveProfile_t g_NodeDriveProfile;

#define MOTION_INVAILD_POS	0x7FFFFFFF
#define MIN_DIFF_DISTANCE   5

#define SCURVE_SPEEDTHRESHOLD  300
#define MOTION_IDLESAMPLETIME  300

MT_MotionDrive_t g_MotionDriveHandle;

static int32_t  MOTION_STATUS = 0;
#define MOTION_RUNNING 		1U
#define MOTION_STOP    		2U
#define MOTION_CORRECTING 	3U

typedef struct MT_RequestBlock_ {
	int32_t  start_pos;
	int32_t  target_pos;
	uint32_t req_time;
	int8_t   direction;
}MT_RequestBlock;

MT_RequestBlock g_MotionRequest;

typedef struct MT_ResultBlock_ {
	int32_t  request_len;    // steps
	int32_t  missed_len;     // steps
	uint32_t estimate_time;  // ms
	uint32_t use_time;       // ms
}MT_ResultBlock;

int32_t MT_ConvertStepsToPos(int32_t steps)
{
	int32_t pos = (steps - g_MotionDriveHandle.homePoint) / ((g_MotionDriveHandle.cwDir==g_MotionDriveHandle.switchdir)?-1:1);
	return pos;
}

void MT_reset()
{
	g_MotionDriveHandle.target = MOTION_INVAILD_POS;

	g_MotionDriveHandle.switchPoint = 0;
	g_MotionDriveHandle.homePoint = 0;

	g_MotionDriveHandle.adMode = MT_ADMode_Normal;
	g_MotionDriveHandle.lock   = 0;
	g_MotionDriveHandle.posQueueMode = MT_PQ_SetAck;
	g_MotionDriveHandle.usePID = 0;

	/*Default settings*/
	g_MotionDriveHandle.reachWindow = 32;
	g_MotionDriveHandle.win1_8 = 4;
	g_MotionDriveHandle.win1_4 = 8;
	g_MotionDriveHandle.win1_2 = 16;

	/*Initialize the profile parameters*/
	MT_setProfile(&g_NodeDriveProfile);

	g_MotionDriveHandle.lock = 0;

	/*Reset the queue and list*/
	if (g_MotionDriveHandle.msgQueue){
		xQueueReset( g_MotionDriveHandle.msgQueue);
	}

	if (g_MotionDriveHandle.posQueue){
		xQueueReset( g_MotionDriveHandle.posQueue);
	}

	g_MotionDriveHandle.state = MT_IDLE;
}

void MT_setProfile(Node_DriveProfile_t* profile)
{
	g_MotionDriveHandle.cwDir = profile->motordir;   // default 0
	g_MotionDriveHandle.ccwDir = (g_MotionDriveHandle.cwDir == 1) ? 0 : 1;  // default 1
	g_MotionDriveHandle.switchdir = profile->switchdir; // default 0
	g_MotionDriveHandle.speed = profile->speed;
}

void MT_Init(QueueHandle_t msgQueue, QueueHandle_t posQueue, QueueHandle_t statusQueue)
{
	MT_reset();
	g_MotionDriveHandle.msgQueue = msgQueue;
	g_MotionDriveHandle.posQueue = posQueue;
	g_MotionDriveHandle.statusQueue = statusQueue;

	g_MotionRequest.direction = Stepper_GetDir();

	/* Initialize position profile model*/
	PosProfile_Initialize();

	/* Initialize pid profile*/
	PIDProfile_Initialize(0.49f, 1.21f, -0.28f, 1.76f);
}

void MT_setReachCallback(void (*pFunct)(int32_t position))
{
	g_MotionDriveHandle.reachCB = pFunct;
}

void MT_setTarget(int32_t target)
{
	if (target == MOTION_INVAILD_POS) {
		configASSERT(0);
		return;
	}

	vPortEnterCritical();
	/*Use target lock mode*/
	g_MotionDriveHandle.target = target;
	vPortExitCritical();
}

/*32 multiples number for window is the best*/
void MT_setReachWindow(int32_t window)
{
	vPortEnterCritical();
	g_MotionDriveHandle.reachWindow = window;
	g_MotionDriveHandle.win1_8 = window / 8;
	g_MotionDriveHandle.win1_4 = window / 4;
	g_MotionDriveHandle.win1_2 = window / 2;
	vPortExitCritical();
}

void MT_setSpeed(uint32_t speed)
{
	vPortEnterCritical();
	g_MotionDriveHandle.speed = speed;
	Stepper_SetSpeed(speed);
	vPortExitCritical();
}

void MT_setLockFlag(int8_t lock)
{
	vPortEnterCritical();
	g_MotionDriveHandle.lock = lock;
	vPortExitCritical();
}

void MT_execFinished(int8_t type)
{
	g_MotionDriveHandle.state = MT_FINISHED;
}


#define REACH_WINDOW_TIME	10

bool MT_checkReach(int32_t diff, uint32_t curTick, int32_t reachWindow)
{
	static uint8_t  reachTimerStart = 0;
	static uint32_t enterWindowTick = 0;
	static uint32_t reachSetTime    = 0;

	bool  reached = false;

	uint8_t  inReachWindow = 0;
	if (abs(diff) <= reachWindow){
		inReachWindow = 1;
	}

	// 1. is in reach window
	if (inReachWindow == 0){
		// Not in reach window
		reachTimerStart = 0;
		enterWindowTick = 0;
		return reached;
	}

	// 2. is reach timer start
	if (reachTimerStart == 0){
		// setup reach timer
		reachTimerStart = 1;

		// save the current tick
		enterWindowTick = curTick;
		reachSetTime    = enterWindowTick + REACH_WINDOW_TIME;
	}

	// 3. is reach window time exceed
	if (curTick >= reachSetTime){
		reached = true;
		reachTimerStart = 0;
		enterWindowTick = 0;
	}

	return reached;
}

void MT_Start(MotionMsgItem_t* pitem, uint32_t ticks)
{
	uint32_t targetVel = 0;
	uint32_t maxVel = 36000;
	uint32_t maxAcce = 28000;
	uint32_t maxJerk = 28000;
	real32_T ts = (real32_T)ticks / 1000.0f;

	/* Build a motion block*/
	g_MotionRequest.start_pos = g_IncSteps;
	g_MotionRequest.direction = Stepper_GetDir();

	if (g_MotionRequest.direction == STEPPER_Dir_CW)
		g_MotionRequest.target_pos = g_MotionRequest.start_pos + pitem->position;
	else
		g_MotionRequest.target_pos = g_MotionRequest.start_pos - pitem->position;

	PosProfile_Start(pitem->position, 0, targetVel, maxVel, maxAcce, maxJerk, ts);

	printf("%ld\n", g_IncSteps);

	Stepper_Run();
	MOTION_STATUS = MOTION_RUNNING;
}

void MT_ProfileStep(uint32_t ticks)
{
	int32_t perr = abs(g_IncSteps - g_MotionRequest.start_pos);
	PosProfile_Step(perr);
	Stepper_SetSpeed(rtY.CmdVel);
}

boolean_T MT_OverTime(uint32_t ticks)
{
	return PosProfile_StepOver(ticks);
}

void MT_Stop()
{
	MOTION_STATUS = MOTION_STOP;
	Stepper_Stop();

	printf("%ld\n", g_IncSteps);
}

boolean_T MT_IsReach()
{
	int32_t diff = abs(g_IncSteps - g_MotionRequest.target_pos);
	if (diff <= 4)
		return true;

	return false;
}

void MT_Correcting()
{
	MOTION_STATUS = MOTION_CORRECTING;
	//printf("correct: %ld\n", g_IncSteps);
	PIDProfile_Start(g_MotionRequest.target_pos, 1200.0f, 1000.0f, 1000.0f);
}

void MT_CorrectingStep(uint32_t ticks)
{
	real32_T ts = ticks/1000.0f;

	PIDProfile_Step(ts, g_IncSteps);

	int32_t vel = rtPIDY.CmdVel;
	int8_t dir = g_MotionRequest.direction;

	if (vel < 0){
		dir = STEPPER_Dir_CCW;
	}else{
		dir = STEPPER_Dir_CW;
	}

	Stepper_SetDir(dir);
	Stepper_SetSpeed(abs(vel));
}

void MT_AddTestBlock()
{
	int8_t dir = g_MotionRequest.direction;
	if (dir == STEPPER_Dir_CW)
		dir = STEPPER_Dir_CCW;
	else
		dir = STEPPER_Dir_CW;
	Stepper_SetDir(dir);
	g_MotionRequest.direction = Stepper_GetDir();

	MotionMsgItem_t item;
	item.position = 24800;
	xQueueSend(g_MotionDriveHandle.msgQueue, &item, 0);
}

void MT_process_v3(uint32_t ticks)
{
	static uint32_t lasttick = 0;
	uint32_t curtick = HAL_GetTick();

	MotionMsgItem_t item;

	/*handle msg queue*/
	if (g_MotionDriveHandle.msgQueue != NULL) {
		BaseType_t result = xQueueReceive(g_MotionDriveHandle.msgQueue, &item, 0);
		if (result == pdPASS) {
			/*Motion plan and run*/
			MT_Start(&item, ticks);
			lasttick = curtick;
			return;
		}
	}

	if (MOTION_STATUS == MOTION_RUNNING){
		MT_ProfileStep(ticks);
		if (MT_OverTime(curtick - lasttick)){
			if (MT_IsReach()){
				MT_Stop();
				MT_AddTestBlock();
			}else{
				MT_Correcting();
			}
		}
	}

	if (MOTION_STATUS == MOTION_CORRECTING){
		MT_CorrectingStep(ticks);
		if (MT_IsReach()){
			MT_Stop();
			MT_AddTestBlock();
		}
	}
}
