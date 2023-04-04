/*
 * node.c
 *
 *  Created on: 2020年9月3日
 *      Author: Administrator
 */


#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"
#include "CANopen.h"
#include "c402.h"
#include "nmotion.h"
#include "mcdrv.h"
#include "log.h"

/*Node global definition*/
extern uint8_t g_NodeId;
uint8_t        CO_NMT_StateChangingSignal = 0;

/*Node driver profile*/
Node_DriveProfile_t g_NodeDriveProfile;
extern MotionCtrlDef_t g_MotionCtrl;

/* return string description of NMT state. */
static char *NmtState2Str(CO_NMT_internalState_t state)
{
    switch(state) {
        case CO_NMT_INITIALIZING:    return "initializing";
        case CO_NMT_PRE_OPERATIONAL: return "pre-operational";
        case CO_NMT_OPERATIONAL:     return "operational";
        case CO_NMT_STOPPED:         return "stopped";
        default:                     return "unknown";
    }
}

void NODE_NMTCallbackPre(void* obj)
{
	/*called by NMT received isr*/
	CO_NMT_StateChangingSignal = 1;
}

void NODE_NMTCallback(CO_NMT_internalState_t state)
{
	printf("Node NM state(%d): %s\n", state, NmtState2Str(state));
	if (state == CO_NMT_OPERATIONAL){
		/*Reset motion*/
		MT_reset();

		/*Reset motion controller*/
		MCD_reset(&g_MotionCtrl);

		/*Push to switch on disabled the mc driver*/
		MCD_setSwtichOnDisabled(&g_MotionCtrl);
	}
}

void NODE_OnMasterHBTimeoutCallback(uint8_t nodeId, uint8_t idx, void *object)
{
	printf("Lost communication with master.\n");
	MT_reset();

	MCD_reset(&g_MotionCtrl);

	CO->NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
}

void NODE_OnMasterHBStartedCallback(uint8_t nodeId, uint8_t idx, void *object)
{
	printf("Master heartbeat started.\n");
}

CO_SDO_abortCode_t NODE_OnProfileUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;

	if (ODF_arg != NULL && ODF_arg->index == 0xC120 && ODF_arg->subIndex == 4) {
		// ds402 profile
		// Update the version
		CO_OD_ROM.ds402Profile.version = (uint32_t)(*(uint32_t*)&ODF_arg->data[0]);

		printf("Node profile update(%08lx)\n", CO_OD_ROM.ds402Profile.version);

		// Update the driver profile
		memcpy(&g_NodeDriveProfile, (unsigned char*)&ODF_arg->data[4], sizeof(Node_DriveProfile_t));

		g_MotionCtrl.profileVel = g_NodeDriveProfile.speed;
		g_MotionCtrl.acce = g_NodeDriveProfile.acce_ratio * g_NodeDriveProfile.speed;
		g_MotionCtrl.dece = g_MotionCtrl.acce;

		g_NodeDriveProfile.updatePositionDuration = 50; // 50ms report actual position
		MCD_OnProfileUpdate(&g_NodeDriveProfile);
	}

	return abort;
}

CO_SDO_abortCode_t NODE_OnControlWordUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
	MCD_OnControlWordUpdate(&g_MotionCtrl);
	return abort;
}

CO_SDO_abortCode_t NODE_OnTargetUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
	MCD_OnTargetUpdate();
	return abort;
}

CO_SDO_abortCode_t NODE_OnModeChange(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
	MCD_changeMode(&g_MotionCtrl);
	return abort;
}

CO_SDO_abortCode_t NODE_OnCommandSerialNumberUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
	MCD_OnCommandSerialNumberUpdate();
	return abort;
}

CO_SDO_abortCode_t NODE_OnVelocityUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
	MCD_OnVelocityUpdate();
	return abort;
}

CO_SDO_abortCode_t NODE_OnAccelerationUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
	MCD_OnAccelerationUpdate();
	return abort;
}

uint8_t NODE_new()
{
	uint32_t COMem;
	CO_ReturnError_t ret = CO_new(&COMem);
	if (ret) {
		LOG_Print(2, "CO new failed: %ld\n", ret);
		return ret;
	}

	return CO_ERROR_NO;
}

uint8_t NODE_Init(void* canDevice, uint8_t nodeId)
{
	CO_ReturnError_t ret;

    ret = CO_CANinit(canDevice, 250);
	if (ret) {
		LOG_Print(2, "CO CAN module init failed: %ld\n", ret);
		return ret;
	}

	ret = CO_CANopenInit(nodeId);
	if (ret) {
		LOG_Print(2, "CO CAN node init failed: %ld\n", ret);
		return ret;
	}

	/*Motion control drive intialize*/
	MCD_Init();

	/* CO node management*/
	/* Implement a simple node management*/
	CO_NMT_initCallbackChanged(CO->NMT, NODE_NMTCallback);
	CO_NMT_initCallbackPre(CO->NMT, NULL, NODE_NMTCallbackPre);

	/* Integrate master heartbeat timeout callback*/
	CO_HBconsumer_initCallbackTimeout(CO->HBcons, 0, NULL, NODE_OnMasterHBTimeoutCallback);

	/* Integrate master heartbeat start callback*/
	CO_HBconsumer_initCallbackHeartbeatStarted(CO->HBcons, 0, NULL, NODE_OnMasterHBStartedCallback);

	/* Register 0xC120 od entry download callback*/
	uint16_t entryNo = CO_OD_find(CO->SDO[0], 0xC120);
	if (entryNo != 0xFFFF){
		CO->SDO[0]->ODExtensions[entryNo].pODFunc = NODE_OnProfileUpdate;
	}

	/* Register 0x6040 od entry control word update callback*/
	entryNo = CO_OD_find(CO->SDO[0], 0x6040);
	if (entryNo != 0xFFFF) {
		CO->SDO[0]->ODExtensions[entryNo].pODFunc = NODE_OnControlWordUpdate;
	}

	entryNo = CO_OD_find(CO->SDO[0], 0x607A);
	if (entryNo != 0xFFFF) {
		CO->SDO[0]->ODExtensions[entryNo].pODFunc = NODE_OnTargetUpdate;
	}

	entryNo = CO_OD_find(CO->SDO[0], 0x6081);
	if (entryNo != 0xFFFF) {
		CO->SDO[0]->ODExtensions[entryNo].pODFunc = NODE_OnVelocityUpdate;
	}

	entryNo = CO_OD_find(CO->SDO[0], 0x6083);
	if (entryNo != 0xFFFF) {
		CO->SDO[0]->ODExtensions[entryNo].pODFunc = NODE_OnAccelerationUpdate;
	}

	entryNo = CO_OD_find(CO->SDO[0], 0xC101);
	if (entryNo != 0xFFFF) {
		CO->SDO[0]->ODExtensions[entryNo].pODFunc = NODE_OnCommandSerialNumberUpdate;
	}
	/* CO node start the CAN driver*/
	CO_CANsetNormalMode(CO->CANmodule[0]);
	if (!CO->CANmodule[0]->CANnormal){
		LOG_Print(2, "CAN module initialize and start failed.\n");
		return CO_ERROR_SYSCALL;
	}

	return CO_ERROR_NO;
}

void NODE_process(uint16_t timeDifference_us, bool_t syncWas)
{
	if (CO_NMT_StateChangingSignal){
		CO_NMT_StateChangingSignal = 0;
		NODE_NMTCallback(CO->NMT->operatingState);
	}

	CO_process_RPDO(CO, true);

	/*Process TPDO*/
	uint32_t next_time;
	CO_process_TPDO(CO, syncWas, timeDifference_us, &next_time);
}
