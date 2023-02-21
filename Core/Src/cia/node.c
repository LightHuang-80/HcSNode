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
#include "mcdrv.h"
#include "log.h"

/*Node global definition*/
extern uint8_t g_NodeId;

/*Node driver profile*/
Node_DriveProfile_t g_NodeDriveProfile;

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

void NODE_NMTCallback(CO_NMT_internalState_t state)
{
	printf("Node NM state(%d): %s\n", state, NmtState2Str(state));
	if (state == CO_NMT_OPERATIONAL){
		/*Should reset the motor*/
		MCD_reset();

		/*Push on the mc driver*/
		MCD_setOn();
	}
}

CO_SDO_abortCode_t NODE_OnProfileUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;

	if (ODF_arg != NULL && ODF_arg->index == 0xC120 && ODF_arg->subIndex == 4) {
		// ds402 profile
		// Update the version
		CO_OD_ROM.ds402Profile.version = (uint32_t)(*(uint32_t*)&ODF_arg->data[0]);

		// Update the driver profile
		memcpy(&g_NodeDriveProfile, (unsigned char*)&ODF_arg->data[4], sizeof(Node_DriveProfile_t));

		g_NodeDriveProfile.updatePositionDuration = 50; // 50ms report actual position
		MCD_OnProfileUpdate(&g_NodeDriveProfile);
	}

	return abort;
}

CO_SDO_abortCode_t NODE_OnControlWordUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
	MCD_OnControlWordUpdate();
	return abort;
}

CO_SDO_abortCode_t NODE_OnTargetUpdate(CO_ODF_arg_t *ODF_arg)
{
	CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
	MCD_OnTargetUpdate();
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


static void Node_processRPDO0(CO_RPDO_t *RPDO)
{
	if (!RPDO->valid || !(*RPDO->operatingState == CO_NMT_OPERATIONAL)) {
        CO_FLAG_CLEAR(RPDO->CANrxNew[0]);
        return;
    }

	uint8_t update = 0;
	for (uint8_t bufNo = 0; bufNo <= 1; bufNo++){
	  if(CO_FLAG_READ(RPDO->CANrxNew[bufNo])){
	    int16_t i;
	    uint8_t* pPDOdataByte;
	    uint8_t** ppODdataByte;

	    i = RPDO->dataLength;
	    pPDOdataByte = &RPDO->CANrxData[bufNo][0];
	    ppODdataByte = &RPDO->mapPointer[0];

	    /* Copy data to Object dictionary. If between the copy operation CANrxNew
	     * is set to true by receive thread, then copy the latest data again. */
	    CO_FLAG_CLEAR(RPDO->CANrxNew[bufNo]);
	    for(; i>0; i--) {
	        **(ppODdataByte++) = *(pPDOdataByte++);
	    }
	    update = 1;
	    break;
	  }
	}

	if (update) {
		MCD_OnControlWordUpdate();
	}
}

void NODE_process(uint16_t timeDifference_us, bool_t syncWas)
{
	Node_processRPDO0(CO->RPDO[0]);

	uint8_t i = 0;
	for (i = 1; i < CO_NO_RPDO; i++) {
	  CO_RPDO_process(CO->RPDO[i], true);
    }

	MCD_broadState(timeDifference_us, syncWas);
}
