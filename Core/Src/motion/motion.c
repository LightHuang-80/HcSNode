/*
 * motion.c
 *
 *  Created on: 2020年10月14日
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
#include "stepper.h"
#include "log.h"
#include "planner.h"
#include "interpolation.h"
#include "motion.h"
#include "mcdrv.h"

extern volatile int32_t    g_IncSteps;
extern Node_DriveProfile_t g_NodeDriveProfile;

#define MOTION_INVAILD_POS	0x7FFFFFFF
#define MIN_DIFF_DISTANCE   5

#define SCURVE_SPEEDTHRESHOLD  300
#define MOTION_IDLESAMPLETIME  300

MT_MotionDrive_t g_MotionDriveHandle;

block_t g_SBlock;

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

	memset(&g_SBlock, 0, sizeof(block_t));
	memset(&g_InterpolationBlock, 0, sizeof(InterpolationBlock_t));

	g_MotionDriveHandle.state = MT_IDLE;
}

void MT_setProfile(Node_DriveProfile_t* profile)
{
	g_MotionDriveHandle.cwDir = profile->motordir;   // default 0
	g_MotionDriveHandle.ccwDir = (g_MotionDriveHandle.cwDir == 1) ? 0 : 1;  // default 1
	g_MotionDriveHandle.switchdir = profile->switchdir; // default 0
	g_InterpolationBlock.vel_to_cw = profile->veltocw; // default 0
	g_MotionDriveHandle.speed = profile->speed;
}

void MT_Init(QueueHandle_t msgQueue, QueueHandle_t posQueue, QueueHandle_t statusQueue)
{
	MT_reset();
	g_MotionDriveHandle.msgQueue = msgQueue;
	g_MotionDriveHandle.posQueue = posQueue;
	g_MotionDriveHandle.statusQueue = statusQueue;
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

void MT_processHomeStart()
{
	LOG_Print(LOG_InfoLevel, "Home start, curinc: %d, dir: %d\n", g_IncSteps, g_MotionDriveHandle.cwDir);

	/*Free style search zero position*/
	g_MotionDriveHandle.lock = 0;
	g_MotionDriveHandle.target = MOTION_INVAILD_POS;

	/*Always set the motor direction firstly*/
	Stepper_SetDir(g_MotionDriveHandle.ccwDir);

	/*Motor start*/
	Stepper_Run();
}

void MT_processGoHome(int32_t offset)
{
	/*Always set the go home direction firstly*/
	Stepper_SetDir(g_MotionDriveHandle.cwDir);
	Stepper_Stop();

	/*Save the zero point*/
	g_MotionDriveHandle.switchPoint = g_IncSteps;

	/*Should calc with switch dir*/
	g_MotionDriveHandle.homePoint = g_IncSteps + ((g_MotionDriveHandle.cwDir==g_MotionDriveHandle.switchdir)?-1:1) * offset;

	/**/
	g_MotionDriveHandle.lock = 1;

	printf("Switch point: %ld, home offset: %ld\n", g_MotionDriveHandle.switchPoint, g_MotionDriveHandle.homePoint);
	MT_setTarget(g_MotionDriveHandle.homePoint);

#if S_CURVE_ACCELERATION == 1

	float acce_ratio = 2.0f;
	float start_speed_factor = 0.1f; // factor <1 decreases speed
	float end_speed_factor = 0.0f; // factor <1 decreases speed

	uint32_t accel = (uint32_t)(g_MotionDriveHandle.speed * acce_ratio);
	g_SBlock.nominal_rate = g_MotionDriveHandle.speed;	// (step/sec) Always > 0
	g_SBlock.acceleration_steps_per_s2 = accel;
	g_SBlock.step_event_count = abs(offset);

	/*Should set the switch dir, ccwDir or cwDir*/
	g_SBlock.direction_bits = g_MotionDriveHandle.cwDir;

	calculate_trapezoid_for_block(&g_SBlock, start_speed_factor, end_speed_factor);
	Stepper_PushBlock(&g_SBlock);
#else
	Stepper_SetSpeed(6400);
#endif
	g_MotionDriveHandle.state = MT_START;
	Stepper_RunMotion(MT_execFinished);
}

void MT_handleMsg(MotionMsgItem_t* item)
{
	switch (item->code) {
	case MMC_HomeStart: {
		MT_processHomeStart();
	}break;

	case MMC_HomeGoHome: {
		int32_t offset = item->position;
		MT_processGoHome(offset);
	}break;

	default:
		break;
	}

	g_MotionDriveHandle.reportReachTarget = g_MotionDriveHandle.lock;
}

void MT_polynomialInterpolation(MotionMsgItem_t* item, int32_t curincsteps)
{
	int32_t target = item->position;

	int32_t motionTarget = g_MotionDriveHandle.homePoint + ((g_MotionDriveHandle.cwDir==g_MotionDriveHandle.switchdir)?-1:1) * target;

	MT_setTarget(motionTarget);

	int32_t lastpos = (curincsteps - g_MotionDriveHandle.homePoint) / ((g_MotionDriveHandle.cwDir==g_MotionDriveHandle.switchdir)?-1:1);

	/*当速度和加速度为0时，一般为最终目标位置，运动完成后锁住位置*/
	if (item->acceleration == 0.0 && item->velocity == 0.0) {
		g_MotionDriveHandle.lock = 1;
	}else {
		g_MotionDriveHandle.lock = 0;
	}

	/*1. degree five caculate
	 * a0, a1, a2, a3, a4, a5 = polynomial(
	 *    current incsteps, target pos,
	 *    last velocity, target velocity,
	 *    last acceleration, target acceleration,
	 *    current tick, duration)*/

	/*采用设置的传输频率，时长 = 1.0/HZ */
	float duration = 0.1;

	taskENTER_CRITICAL();
	g_InterpolationBlock.enable = 0;
	INP_calc_coeffs(&g_InterpolationBlock, lastpos, target, item->velocity, item->acceleration, duration);

	/*2. 设置起始速度，开始stepper*/
	INP_Prepare();
	taskEXIT_CRITICAL();
}

void MT_scurveInterpolation(MotionMsgItem_t* item, int32_t curincsteps, float start_speed_factor, float end_speed_factor)
{
	/*当启动速度，启动加速度和停止速度、减速度都为0时，采用S-Curve*/
	int32_t target = item->position;

	target = g_MotionDriveHandle.homePoint + ((g_MotionDriveHandle.cwDir==g_MotionDriveHandle.switchdir)?-1:1) * target;
	MT_setTarget(target);

	float acce_ratio = g_NodeDriveProfile.acce_ratio;

	//float start_speed_factor = 0.15f; // factor <1 decreases speed
	//float end_speed_factor = 0.0f;    // factor <1 decreases speed

	/*设置速度*/
	if (g_MotionDriveHandle.speed < SCURVE_SPEEDTHRESHOLD)
		g_MotionDriveHandle.speed = SCURVE_SPEEDTHRESHOLD;

	uint32_t accel = (uint32_t)(g_MotionDriveHandle.speed * acce_ratio);

	g_SBlock.nominal_rate = g_MotionDriveHandle.speed;	// (step/sec) Always > 0
	g_SBlock.acceleration_steps_per_s2 = accel;

	/*should calc with switch dir*/
	if (g_MotionDriveHandle.cwDir != g_MotionDriveHandle.switchdir) {
		if (curincsteps < target) {
			g_SBlock.direction_bits = g_MotionDriveHandle.cwDir;
		}else{
			g_SBlock.direction_bits = g_MotionDriveHandle.ccwDir;
		}
	}else{
		if (curincsteps < target) {
			g_SBlock.direction_bits = g_MotionDriveHandle.ccwDir;
		}else{
			g_SBlock.direction_bits = g_MotionDriveHandle.cwDir;
		}
	}

	g_SBlock.step_event_count =  abs(curincsteps - target);
	g_MotionDriveHandle.lock = 1;

	calculate_trapezoid_for_block(&g_SBlock, start_speed_factor, end_speed_factor);
	Stepper_PushBlock(&g_SBlock);
}

void MT_handleMotion(MotionMsgItem_t* item, int32_t curincsteps)
{
	if (item->type == 0){
		/*S-Curve interpolation*/
		MT_scurveInterpolation(item, curincsteps, 0.15f, 0.0f);
	}else {
		MT_polynomialInterpolation(item, curincsteps);
	}
	g_MotionDriveHandle.state = MT_START;
	Stepper_RunMotion(MT_execFinished);
	g_MotionDriveHandle.reportReachTarget = g_MotionDriveHandle.lock;
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


void MT_process_v2(int32_t incsteps)
{
	static uint32_t lasttick = 0;
	uint32_t curtick = HAL_GetTick();

	static uint32_t elapsetime = 0;
	static uint32_t exhausttime = 0;

	MotionMsgItem_t item;

	/*handle msg queue*/
	if (g_MotionDriveHandle.msgQueue != NULL) {
		BaseType_t result = xQueueReceive(g_MotionDriveHandle.msgQueue, &item, 0);
		if (result == pdPASS) {
			lasttick = curtick;

			MT_handleMsg(&item);
			return;
		}
	}

	if (g_MotionDriveHandle.posQueue != NULL) {
		/*运行中模式, 在稳速运行时检查当前位置和步数误差
		 * Delta = Motor run steps - (current incsteps - home point)*/
		BaseType_t result;

		if (curtick >= elapsetime) {
		  result = xQueueReceive(g_MotionDriveHandle.posQueue, &item, 0);
		  if (result == pdPASS) {
			lasttick = curtick;

			/*next block section，时长为发送频率 1000ms/HZ*/
			elapsetime  = curtick + 100;
			exhausttime = curtick + 400;

			LOG_Print(LOG_DbgLevel, "[%d](%d)cur:%d, tgt:%d, v:%d, a:%d, dt:%d\n",
					curtick,
					item.type,
					incsteps ,
					item.position,(int32_t)ceilf(item.velocity), (int32_t)ceilf(item.acceleration),
					item.duration);

			MT_handleMotion(&item, incsteps);
			return;
		  }
		}

		/*超过400ms没有收到motion msg时，采用lock方式回到目标位置*/
		if (curtick >= exhausttime){
			if (g_MotionDriveHandle.lock == 0) {
				g_MotionDriveHandle.lock = 1;
			}
		}
	}

	if (g_MotionDriveHandle.lock == 1) {

		/*reach the target point*/
		if (g_MotionDriveHandle.target == MOTION_INVAILD_POS){
			lasttick = curtick;
			return;
		}

		int32_t diff =  incsteps - g_MotionDriveHandle.target;

		// 检查是否reach target
		if (MT_checkReach(diff, curtick, g_MotionDriveHandle.reachWindow / 2) == 1){
			lasttick = curtick;
			Stepper_Stop();

			//LOG_Print(LOG_InfoLevel, "Target(%d) reached, diff: %d\n", g_MotionDriveHandle.target, diff);
			/*Reach the target, set status reach bit[b:10]*/
			if (g_MotionDriveHandle.reachCB != NULL){
				g_MotionDriveHandle.reachCB(incsteps);
			}

			if (g_MotionDriveHandle.reportReachTarget == 1){
				g_MotionDriveHandle.reportReachTarget = 0;
				// setup the reach flag
				// push a reach message
			    StatusMsgItem_t item;
			    item.code = MMC_TargetReach;
			    xQueueSend(g_MotionDriveHandle.statusQueue, &item, 0);
			}

			return;
		}

		/*运行时间间隔检查, PID*/
		int32_t tickdiff = curtick - lasttick;
		if (tickdiff > MOTION_IDLESAMPLETIME && g_MotionDriveHandle.state == MT_FINISHED) {
			lasttick = curtick;
			//LOG_Print(LOG_InfoLevel, "[%d]diff: %d, curinc: %d, target: %d\n", curtick, diff,  incsteps, g_MotionDriveHandle.target);

			if (abs(diff) > g_MotionDriveHandle.reachWindow) {

				/*定位设置*/

				if (g_MotionDriveHandle.cwDir != g_MotionDriveHandle.switchdir) {
					if (incsteps < g_MotionDriveHandle.target) {
						Stepper_SetDir(g_MotionDriveHandle.cwDir);
					}else{
						Stepper_SetDir(g_MotionDriveHandle.ccwDir);
					}
					//diff *= -1;
				}else{
					if (incsteps < g_MotionDriveHandle.target) {
						Stepper_SetDir(g_MotionDriveHandle.ccwDir);
					}else{
						Stepper_SetDir(g_MotionDriveHandle.cwDir);
					}
				}

				Stepper_SetSpeed(400);
				Stepper_Run();
				//Stepper_RunSteps(diff);
/*
				item.acceleration = g_NodeDriveProfile.acce_ratio;
				item.velocity = g_NodeDriveProfile.speed;
				item.position = MT_ConvertStepsToPos(g_MotionDriveHandle.target);

				float start_speed_factor = 0.1;
				uint32_t realspeed = Stepper_GetRealSpeed();
				start_speed_factor = realspeed / item.velocity;
				MT_scurveInterpolation(&item, incsteps, start_speed_factor, 0.0f);*/
			}
		}
	}
}
