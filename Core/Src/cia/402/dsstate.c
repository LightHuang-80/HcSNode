/*
 * dsstate.c
 *
 *  Created on: 2020��5��26��
 *      Author: Administrator
 */

#include "c402def.h"

/*
void DS402_StateManage_Init(void)
{
	g_DSStateData.operate = DS402_OP_Donothing;
	g_DSStateData.result = DS402_OPR_OK;

	g_DSStateData.triggerdCommand = DS402_Command_None;

	g_DSStateData.lastState = DS402_Status_SwitchONDisabled;
	g_DSStateData.state = DS402_Status_SwitchONDisabled;

	g_DSStateData.lastStatusWord = DS402_ChangeStatusWord(0xFFFF, DS402_Status_SwitchONDisabled);
	g_DSStateData.statusWord = g_DSStateData.lastStatusWord;

	//Voltage Enabled
	DS402_SetStatusWordBit(&g_DSStateData.statusWord, DS402_SWB_VoltageEnabled, 1);

	//Reset warning
	DS402_SetStatusWordBit(&g_DSStateData.statusWord, DS402_SWB_Warning, 0);

	//ManufacturerSpecific not used
	DS402_SetStatusWordBit(&g_DSStateData.statusWord, DS402_SWB_ManufacturerSpecific, 0);

	//Enable remote control
	DS402_SetStatusWordBit(&g_DSStateData.statusWord, DS402_SWB_Remote, 1);

	//Target reached, set by driver
	DS402_SetStatusWordBit(&g_DSStateData.statusWord, DS402_SWB_TargetReached, 0);

	//Bit 11, software limit set
	DS402_SetStatusWordBit(&g_DSStateData.statusWord, DS402_SWB_InternalLimitActive, 0);

	//Bits 12/13
	DS402_SetStatusWordBit(&g_DSStateData.statusWord, DS402_SWB_OperationModeSpecific0, 0);
	DS402_SetStatusWordBit(&g_DSStateData.statusWord, DS402_SWB_OperationModeSpecific1, 1);

	g_DSStateData.lastStatusWord = g_DSStateData.statusWord;
}*/

uint16_t DS402_buildStatusWordByStatus(uint16_t curStatusWord, DS402_Status_t status)
{
	uint16_t newStatusWord = curStatusWord;

	switch(status){
	case DS402_Status_NotReadyToSwitchON:
		newStatusWord &= 0xFFB0;
		break;
	case DS402_Status_SwitchONDisabled:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0040;
		break;
	case DS402_Status_ReadyToSwitchON:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0021;
		break;
	case DS402_Status_SwitchedON:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0023;
		break;
	case DS402_Status_OperationEnable:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0027;
		break;
	case DS402_Status_QuickStopActive:
		newStatusWord &= 0xFF90;
		newStatusWord |= 0x0007;
		break;
	case DS402_Status_FaultReactionActive:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x000F;
		break;
	case DS402_Status_Fault:
		newStatusWord &= 0xFFB0;
		newStatusWord |= 0x0008;
		break;
	default:
		break;
	}

	return newStatusWord;
}

void DS402_SetStatusWordBit(uint16_t* statusWord, StatusWordBit_t bit, uint8_t enable){
	if (bit >= 16) return;

	uint16_t bitset = 1 << bit;
	if (enable == 1)
		*statusWord |= bitset;
	else
		*statusWord &= ~bitset;
}

uint16_t DS402_buildControlWordByCommand(uint16_t controlWord, DS402_Command_t command)
{
	if (command == DS402_Command_Shutdown) {
		controlWord &= 0xFF7E;
		controlWord |= 0x0006;
	}else if (command == DS402_Command_SwitchON) {
		controlWord &= 0xFF70;
		controlWord |= 0x0007;
	}else if (command == DS402_Command_EnableOperation) {
		controlWord &= 0xFF70;
		controlWord |= 0x000F;
	}

	return controlWord;
}

DS402_Status_t DS402_getStatus(uint16_t statusWord)
{
	DS402_Status_t status = DS402_Status_Fault;
	uint16_t sw = statusWord & 0x6F;

	if (sw == 0x21) {
		status = DS402_Status_ReadyToSwitchON;
	}else if (sw == 0x23) {
		status = DS402_Status_SwitchedON;
	}else if (sw == 0x27) {
		status = DS402_Status_OperationEnable;
	}else if (sw == 0x07) {
		status = DS402_Status_QuickStopActive;
	}else {
		sw &= 0x4F;
		if (sw == 0) {
			status = DS402_Status_NotReadyToSwitchON;
		}else if (sw == 0x40) {
			status = DS402_Status_SwitchONDisabled;
		}else if (sw == 0x0F) {
			status = DS402_Status_FaultReactionActive;
		}
	}

	return status;
}

DS402_Command_t DS402_getCommandFromControlWord(uint16_t controlWord)
{
	DS402_Command_t command = DS402_Command_None;

	if ((controlWord & 0x000F) == 0x000F) {
		command = DS402_Command_EnableOperation;
	}else if ((controlWord & 0x0007) == 0x0007 ) {
		command = DS402_Command_SwitchON;
	}else if ((controlWord & 0x0006) == 0x0006) {
		command = DS402_Command_Shutdown;
	}else if ((controlWord & 0x0002) == 0x0002) {
		command = DS402_Command_QuickStop;
	}else {
		if ((controlWord & 0x0002) == 0){
			command = DS402_Command_DisableVoltage;
		}
	}
	return command;
}

DS402_Status_t DS402_getNewStatusByControlWord(uint16_t statusWord, uint16_t controlWord)
{
	DS402_Status_t newStatus = DS402_Status_Fault;
	DS402_Status_t curStatus = DS402_getStatus(statusWord);

	uint32_t cw = controlWord & 0x0F;
	    if ((cw & 0x02) == 0){
	    	if (curStatus == DS402_Status_ReadyToSwitchON ||
	    		curStatus == DS402_Status_OperationEnable ||
				curStatus == DS402_Status_SwitchedON ||
				curStatus == DS402_Status_QuickStopActive){
	    		newStatus = DS402_Status_SwitchONDisabled;
	    	}
	    }

	    if (cw == 0x06 || cw == 0x0E){
	    	if (curStatus == DS402_Status_SwitchONDisabled ||
	    		curStatus == DS402_Status_SwitchedON ||
				curStatus == DS402_Status_OperationEnable){
	    		newStatus = DS402_Status_ReadyToSwitchON;
	    	}
	    }else if (cw == 0x07){
	    	if (curStatus == DS402_Status_ReadyToSwitchON ||
	    		curStatus == DS402_Status_SwitchONDisabled){
	    		newStatus = DS402_Status_SwitchedON;
	    	}else if (curStatus == DS402_Status_OperationEnable){
	    		newStatus = DS402_Status_SwitchedON;
	    	}
	    }else if (cw == 0x0F){
	    	if (curStatus == DS402_Status_ReadyToSwitchON){
	    		newStatus = DS402_Status_SwitchedON;
	    	}else if (curStatus == DS402_Status_SwitchedON ||
	    			curStatus == DS402_Status_QuickStopActive ||
					curStatus == DS402_Status_OperationEnable){
	    		newStatus = DS402_Status_OperationEnable;
	    	}
	    }else if (cw == 0x02 || cw == 0x03){
	    	if (curStatus == DS402_Status_ReadyToSwitchON ||
	    		curStatus == DS402_Status_SwitchedON){
	    		newStatus = DS402_Status_SwitchONDisabled;
	    	}else if (curStatus == DS402_Status_OperationEnable){
	    		newStatus = DS402_Status_QuickStopActive;
	    	}
	    }
	return newStatus;
}

DS402_OperMode_Action_t DS402_getModeActionFromControlWord(int8_t mode, uint16_t controlWord)
{
	DS402_OperMode_Action_t action = DS402_OperMode_Halt;

	switch (mode) {
	case DS402_OperMode_Homing: {
		if ((controlWord & 0x10) == 0x10){
			action = DS402_OperMode_HomingStart;
		}
	}break;
	case DS402_OperMode_ProfilePosition: {
		if ((controlWord & 0x10) == 0x10){
			action = DS402_OperMode_NewSetPoint;
		}else if ((controlWord & 0x20) == 0x20) {
			action = DS402_OperMode_ChangeSetImmediately;
		}
	}break;
	case DS402_OperMode_InterpolatedPosition: {
		if ((controlWord & 0x10) == 0x10){
			action = DS402_OperMode_EnableIP;
		}
	}break;

	default:
		break;
	}

	return action;
}

/*
static DS402_OperateResult_t DS402_ChangeState(DS402_StateData_t *data)
{
	DS402_OperateResult_t result = DS402_OPR_StateNotChanged;

	DS402_Command_t command = DS402_Command_None;
	DS402_Status_t  state = data->state;

    uint32_t cw = data->newControlWord & 0x0F;
    if ((cw & 0x02) == 0){
    	if (state == DS402_Status_ReadyToSwitchON ||
    		state == DS402_Status_OperationEnable ||
			state == DS402_Status_SwitchedON ||
			state == DS402_Status_QuickStopActive){
    		command = DS402_Command_DisableVoltage;
    		state = DS402_Status_SwitchONDisabled;
    	}
    }

    if (cw == 0x06 || cw == 0x0E){
    	if (state == DS402_Status_SwitchONDisabled ||
    		state == DS402_Status_SwitchedON ||
			state == DS402_Status_OperationEnable){
    		state = DS402_Status_ReadyToSwitchON;
    		command = DS402_Command_Shutdown;
    	}
    }else if (cw == 0x07){
    	if (state == DS402_Status_ReadyToSwitchON ||
    		state == DS402_Status_SwitchONDisabled){
    		state = DS402_Status_SwitchedON;
    		command = DS402_Command_SwitchON;
    	}else if (state == DS402_Status_OperationEnable){
    		state = DS402_Status_SwitchedON;
    		command = DS402_Command_DisableOperation;
    	}
    }else if (cw == 0x0F){
    	if (state == DS402_Status_ReadyToSwitchON){
    		state = DS402_Status_SwitchedON;
    		command = DS402_Command_SwitchON;
    	}else if (state == DS402_Status_SwitchedON ||
    			state == DS402_Status_QuickStopActive){
    		state = DS402_Status_OperationEnable;
    		command = DS402_Command_EnableOperation;
    	}
    }else if (cw == 0x02 || cw == 0x03){
    	if (state == DS402_Status_ReadyToSwitchON ||
    		state == DS402_Status_SwitchedON){
    		state = DS402_Status_SwitchONDisabled;
    		command = DS402_Command_QuickStop;
    	}else if (state == DS402_Status_OperationEnable){
    		state = DS402_Status_QuickStopActive;
    		command = DS402_Command_QuickStop;
    	}
    }

    data->controlWord = data->newControlWord;

    if (state != data->state){
    	data->lastState = data->state;
    	data->state = state;

    	data->triggerdCommand = command;
    	result = DS402_OPR_StateChanged;

    	uint16_t statusWord = data->statusWord;
    	data->statusWord = DS402_ChangeStatusWord(statusWord, state);

    	data->lastStatusWord = statusWord;
    }

    return result;
}*/
