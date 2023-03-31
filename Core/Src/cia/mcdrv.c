/*
 * mcdrv.c
 *
 *  Created on: 2020年10月5日
 *      Author: Administrator
 */
#include <stdio.h>
#include <math.h>

#include "CANopen.h"
#include "CO_OD.h"
#include "c402def.h"
#include "c402.h"
#include "dsstate.h"
#include "dspp.h"
#include "home.h"
#include "trajectory.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "log.h"
#include "nmotion.h"
#include "mcdrv.h"

#define MOTION_UNKNOWN_MODE   0
#define MOTION_DISCRETE_MODE  1
#define MOTION_QUEUED_MODE    2
#define MOTION_COMPLEX_MODE   3

#define SAMEPOS_RESENT        3

uint8_t g_MotionType;

extern MotionCtrlDef_t   g_MotionCtrl;

extern Node_DriveProfile_t g_NodeDriveProfile;
extern volatile int32_t  g_IncSteps;

void MCD_reachHome(MotionCtrlDef_t* pmc, int32_t offset)
{
	// LOG_Print(LOG_InfoLevel, "Home offset from zero point: %ld\n", offset);
	int16_t ret = (1 << 10) | (1 << 12);
	pmc->statusWord |= ret;
}

void MCD_reset(MotionCtrlDef_t* pmc)
{
	g_MotionType = MOTION_UNKNOWN_MODE;

	pmc->statusWord = DS402_buildStatusWordByStatus(pmc->statusWord, DS402_Status_NotReadyToSwitchON);
	pmc->controlWord = 0;

	pmc->mode = DS402_OperMode_NoMode;
	pmc->displayMode = DS402_OperMode_NoMode;

	HOME_reset();
}

/*Called by node's operational routine*/
void MCD_setSwtichOnDisabled(MotionCtrlDef_t* pmc)
{
	pmc->statusWord = DS402_buildStatusWordByStatus(pmc->statusWord, DS402_Status_SwitchONDisabled);
}

void MCD_Init(MotionCtrlDef_t* pmc)
{
	MCD_reset(pmc);

	/*Initialize the home, pp, ip mode callback*/
	HOME_Init(g_NodeDriveProfile.home_offset, 17, g_NodeDriveProfile.home_speed, 4*g_NodeDriveProfile.home_speed);

	/*Profile position model initialize*/
	PP_Init();

	/*Trajectory position blocks*/
	TPB_Init();
	pmc->statusWord = DS402_buildStatusWordByStatus(pmc->statusWord, DS402_Status_NotReadyToSwitchON);
}


/* Motor control driver, change mode
 * mode: received from host
 * displaymode: set right mode if initialize successfully */
void MCD_changeMode(MotionCtrlDef_t* pmc)
{
	if (pmc->mode == DS402_OperMode_Homing ) {
		HOME_active();
	}else if (pmc->mode == DS402_OperMode_ProfilePosition){
		PP_active(pmc->controlWord);
	}
	pmc->displayMode = pmc->mode;
}

void MCD_OnProfileUpdate(Node_DriveProfile_t *profile)
{
	/*Reset the home and pp*/
	HOME_setProfile(profile->home_offset, 17, profile->home_speed, 4*profile->home_speed);
	MT_setProfile(profile);
}

void MCD_handleModeAction(MotionCtrlDef_t* pmc)
{
	if (pmc->displayMode == DS402_OperMode_Homing){
		HOME_exec(pmc->controlWord);
		/*Bit 10, target reached*/
		/*Bit 12, home attained*/
		/*Bit 13, homing error happen*/
	}else if (pmc->displayMode == DS402_OperMode_ProfilePosition){
		/*Set motion type, control : 0x001f(Discrete|queued mode), 0x021f(Complex queued mode)*/
		/*Bit 10, target reached*/
		/*Bit 12 Set-point acknowledge */

		/*Bit 13 Following error  */
		PP_exec(pmc->controlWord);
	}
}

void MCD_changeStatus(MotionCtrlDef_t* pmc, DS402_Status_t status)
{
	pmc->statusWord = DS402_buildStatusWordByStatus(pmc->statusWord, status);
}

void MCD_OnControlWordUpdate(MotionCtrlDef_t* pmc)
{
	DS402_Status_t curStatus = DS402_getStatus(pmc->statusWord);

	DS402_Status_t newStatus = DS402_getNewStatusByControlWord(curStatus, pmc->controlWord);
	if (newStatus != curStatus) {
		/*Change current status*/
		MCD_changeStatus(pmc, newStatus);
	}

	if (newStatus == DS402_Status_OperationEnable) {

		if (pmc->mode != pmc->displayMode) {
			/*Change current mode*/
			MCD_changeMode(pmc);
		}

		MCD_handleModeAction(pmc);
	}
}

void MCD_OnCommandSerialNumberUpdate()
{
	//int32_t commandSerialNumber = g_MotorDriverData.commandSerialNumber;
	//TPB_commandSerialNumberUpdate(commandSerialNumber);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

void MCD_OnTargetUpdate()
{
	//printf("Received the target pos: %ld\n", g_MotionCtrl.targetPos);
	//float ftarget = *(float*)&g_MotorDriverData.targetPos;
	//TPB_positionUpdate(ftarget);
}

void MCD_OnVelocityUpdate()
{
	//float fvelocity = *(float*)&g_MotorDriverData.profileVel;
	//TPB_velocityUpdate(fvelocity);
}

void MCD_OnAccelerationUpdate()
{
	//float facceleration = *(float*)&g_MotorDriverData.acce;
	//TPB_accelerationUpdate(facceleration);
}

#pragma GCC diagnostic pop
