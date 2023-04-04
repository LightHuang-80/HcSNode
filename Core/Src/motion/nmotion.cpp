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
#include "c402def.h"
#include "c402.h"
#include "nstepper.h"
#include "log.h"
#include "planner.h"

#include "nmotion.h"
#include "mcdrv.h"

#include "motor.h"
#include "dspp.h"

extern volatile int32_t    g_IncSteps;
extern Node_DriveProfile_t g_NodeDriveProfile;
extern MotionCtrlDef_t     g_MotionCtrl;

MT_MotionDrive_t MotionDriveHandle;

void MT_reset()
{
	/*Initialize the profile parameters*/
	MT_setProfile(&g_NodeDriveProfile);

	MotionDriveHandle.prevControlword = 0;
	MotionDriveHandle.mode = DS402_OperMode_NoMode;
}

void MT_begin()
{
	MT_reset();
	HAL_TIM_Base_Start_IT(&htim5);
}

void MT_loop(uint32_t ticks)
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
}

void MT_init()
{
	MT_reset();

	/* Initialize position profile model*/
	PosProfile_Initialize();

	/* Initialize pid profile*/
	PIDProfile_Initialize(0.49f, 1.21f, -0.28f, 1.76f);
}

int8_t MT_getMode()
{
	return MotionDriveHandle.mode;
}

void MT_setMode(int8_t mode)
{
	switch(mode){
	case DS402_OperMode_ProfilePosition:
		PP_active();
		break;
	}
	MotionDriveHandle.mode = mode;
}

void MT_exec(uint16_t controlword)
{
	switch(MotionDriveHandle.mode){
	case DS402_OperMode_ProfilePosition:{
		uint8_t setNewpoint = 0;
		if ((MotionDriveHandle.prevControlword & 0x10) == 0 && (controlword & 0x10))
			setNewpoint = 1;
		PP_exec(controlword, setNewpoint);
	}break;
	}
	MotionDriveHandle.prevControlword = controlword;
}

void MT_process_v3(uint32_t ticks)
{
	switch(MotionDriveHandle.mode){
	case DS402_OperMode_ProfilePosition:
		PP_process(ticks);
		break;
	}
}

