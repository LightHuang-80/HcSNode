/*
 * dsstate.h
 *
 *  Created on: 2020��5��27��
 *      Author: Administrator
 */

#ifndef DSSTATE_H_
#define DSSTATE_H_

void DS402_StateManage_Init(void);
void DS402_StateManage_Process(void);

uint16_t DS402_buildStatusWordByStatus(uint16_t curStatusWord, DS402_Status_t state);
uint16_t DS402_buildControlWordByCommand(uint16_t controlWord, DS402_Command_t command);

DS402_Command_t 			DS402_getCommandFromControlWord(uint16_t controlWord);
DS402_Status_t 				DS402_getNewStatusByControlWord(uint16_t statusWord, uint16_t controlWord);
DS402_OperMode_Action_t 	DS402_getModeActionFromControlWord(int8_t mode, uint16_t controlWord);
DS402_Status_t 				DS402_getStatus(uint16_t statusWord);

#endif /* DSSTATE_H_ */
