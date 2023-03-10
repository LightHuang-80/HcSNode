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

extern MotorDriverData_t g_MotorDriverData;
extern QueueHandle_t     g_TargetPosQueue;
extern QueueHandle_t     g_StatusMsgQueue;

extern Node_DriveProfile_t g_NodeDriveProfile;
extern volatile int32_t  g_IncSteps;

void MCD_reachHome(int32_t offset)
{
	// LOG_Print(LOG_InfoLevel, "Home offset from zero point: %ld\n", offset);

	int16_t ret = (1 << 10) | (1 << 12);
	g_MotorDriverData.statusWord |= ret;
}

void MCD_reachTarget(int32_t position)
{
	// LOG_Print(LOG_InfoLevel, "PP reach the target: %ld\n", position);
	int16_t ret = (1 << 10);
	g_MotorDriverData.statusWord |= ret;
}

void MCD_reportActualPosition(uint16_t timeDifference_us, int32_t steps)
{
	if (g_NodeDriveProfile.ratio == 0)
		return;

	if (g_NodeDriveProfile.updatePositionDuration == 0){
		/* Not update actual position*/
		return;
	}

	static uint32_t ticks = 0;
	static uint16_t samepos_resent = 0;
    static float    lastradpos = 0.0f;

    ticks += timeDifference_us / 1000;

    if (ticks < g_NodeDriveProfile.updatePositionDuration)
    	return;

    ticks = 0;

	int32_t pos = MT_ConvertStepsToPos(steps);

	float   radpos = (pos - g_NodeDriveProfile.lower_limit_range) / g_NodeDriveProfile.ratio;
	if (fabsf(lastradpos - radpos) <= 0.0004){
    	if (++samepos_resent >= SAMEPOS_RESENT)
            return;
    }else{
    	samepos_resent = 0;
    }
    lastradpos = radpos;

	/* Update the 0x6063 actual position*/
	memcpy((void*)&g_MotorDriverData.currentPos, &radpos, sizeof(int32_t));

	bool_t syncWas = true;
	CO->TPDO[1]->sendRequest = true;

	/* TPDO 1 channel, 0x6063 sdo*/
	CO_TPDO_process(CO->TPDO[1], syncWas, timeDifference_us, NULL);
}

void MCD_reset()
{
	g_MotionType = MOTION_UNKNOWN_MODE;

	g_MotorDriverData.statusWord = DS402_buildStatusWordByStatus(g_MotorDriverData.statusWord, DS402_Status_SwitchONDisabled);

	g_MotorDriverData.mode = DS402_OperMode_NoMode;
	g_MotorDriverData.displayMode = DS402_OperMode_NoMode;

	HOME_reset();
}

/*Called by node's operational routine*/
void MCD_setOn()
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	StatusMsgItem_t item;
	item.code = MMC_StatusUpdate;
	xQueueSendFromISR(g_StatusMsgQueue, &item, &xHigherPriorityTaskWokenByPost);

	if(xHigherPriorityTaskWokenByPost == pdTRUE) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
	}
}

void MCD_Init()
{
	MCD_reset();

	/*Initialize the home, pp, ip mode callback*/
	HOME_Init(g_NodeDriveProfile.home_offset, 17, g_NodeDriveProfile.home_speed, 4*g_NodeDriveProfile.home_speed);
	HOME_setFinishedCallback(MCD_reachHome);

	PP_Init(g_NodeDriveProfile.speed);
	PP_setTargetReachedCallback(MCD_reachTarget);

	/*Trajectory position blocks*/
	TPB_Init();

	g_MotorDriverData.statusWord = DS402_buildStatusWordByStatus(g_MotorDriverData.statusWord, DS402_Status_NotReadyToSwitchON);
}

void MCD_uploadStatus(uint16_t timeDifference_us,  bool_t syncWas, uint16_t status)
{
	uint32_t timerNext;
	static uint16_t lastUploadStatus = 0;
	static uint16_t ticks = 0;

	if (lastUploadStatus != status){
		/*upload right now*/
		CO_TPDO_process(CO->TPDO[0], syncWas, timeDifference_us, &timerNext);
		lastUploadStatus = status;
		ticks = 0;
		return;
	}

	ticks += timeDifference_us / 1000;
	if (ticks >= 60){
		ticks = 0;
		/*Upload the status*/
		CO_TPDO_process(CO->TPDO[0], syncWas, timeDifference_us, &timerNext);
		lastUploadStatus = status;
	}
}

void MCD_broadState(uint16_t timeDifference_us, bool_t syncWas)
{
	BaseType_t result;
	StatusMsgItem_t item;
    result = xQueueReceive(g_StatusMsgQueue, &item, 0);
	if (result == pdTRUE){
		// report the status message
		CO->TPDO[0]->sendRequest = true;
		CO_TPDOsend(CO->TPDO[0]);
        return;
	}

	if (g_MotionType != MOTION_UNKNOWN_MODE){
		MCD_reportActualPosition(timeDifference_us, g_IncSteps);
	}
}

/* Motor control driver, change mode
 * mode: received from host
 * displaymode: set right mode if initialize successfully */
void MCD_changeMode(int8_t mode)
{
	if (mode == DS402_OperMode_Homing ) {
		g_MotorDriverData.displayMode = mode;

		HOME_active();
	}else if (mode == DS402_OperMode_ProfilePosition){
		g_MotorDriverData.statusWord &= ~0x1400;
		g_MotorDriverData.displayMode = mode;

		PP_active();
	}
}

void MCD_OnProfileUpdate(Node_DriveProfile_t *profile)
{
	/*Reset the home and pp*/
	HOME_setProfile(profile->home_offset, 17, profile->home_speed, 4*profile->home_speed);
	PP_setSpeed(profile->speed);
	MT_setProfile(profile);
}

void MCD_handleModeAction(uint16_t controlWord)
{
	if (g_MotorDriverData.displayMode == DS402_OperMode_Homing){
		uint16_t ret = HOME_exec(controlWord);

		/*Bit 10, target reached*/
		/*Bit 12, home attained*/
		/*Bit 13, homing error happen*/
		g_MotorDriverData.statusWord |= ret;
	}else if (g_MotorDriverData.displayMode == DS402_OperMode_ProfilePosition){
		/*Set motion type, control : 0x001f(Discrete|queued mode), 0x021f(Complex queued mode)*/
		uint16_t mode = controlWord & 0x021f;

		if (mode == 0x021f){
			g_MotionType = MOTION_COMPLEX_MODE;
		}else if (mode == 0x1f){
			g_MotionType = MOTION_QUEUED_MODE;
		}else {
			if ((mode & 0x0f) != 0x0f){
				g_MotionType = MOTION_UNKNOWN_MODE;
			}
		}

		/*Bit 10, target reached*/
		/*Bit 12 Set-point acknowledge */
		/*Bit 13 Following error  */
		uint16_t ret = PP_exec(controlWord);
		g_MotorDriverData.statusWord |= ret;
	}
}

void MCD_changeStatus(DS402_Status_t status)
{
	g_MotorDriverData.statusWord = DS402_buildStatusWordByStatus(g_MotorDriverData.statusWord, status);
}

void MCD_OnControlWordUpdate()
{
	DS402_Status_t curStatus = DS402_getStatus(g_MotorDriverData.statusWord);

	DS402_Status_t newStatus = DS402_getNewStatusByControlWord(g_MotorDriverData.statusWord, g_MotorDriverData.controlWord);
	if (newStatus != curStatus) {
		/*Change current status*/
		MCD_changeStatus(newStatus);
	}

	if (newStatus == DS402_Status_OperationEnable) {
		if (g_MotorDriverData.mode != g_MotorDriverData.displayMode) {
			/*Change current mode*/
			MCD_changeMode(g_MotorDriverData.mode);
		}else {
			MCD_handleModeAction(g_MotorDriverData.controlWord);
		}
	}

	/*Always uploads the status after control word update*/
	StatusMsgItem_t item;
	item.code = MMC_StatusUpdate;
	xQueueSend(g_StatusMsgQueue, &item, 0);
}

void MCD_OnCommandSerialNumberUpdate()
{
	int32_t commandSerialNumber = g_MotorDriverData.commandSerialNumber;
	TPB_commandSerialNumberUpdate(commandSerialNumber);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

void MCD_OnTargetUpdate()
{
	float ftarget = *(float*)&g_MotorDriverData.targetPos;
	TPB_positionUpdate(ftarget);
}

void MCD_OnVelocityUpdate()
{
	float fvelocity = *(float*)&g_MotorDriverData.profileVel;
	TPB_velocityUpdate(fvelocity);
}

void MCD_OnAccelerationUpdate()
{
	float facceleration = *(float*)&g_MotorDriverData.acce;
	TPB_accelerationUpdate(facceleration);
}

#pragma GCC diagnostic pop
