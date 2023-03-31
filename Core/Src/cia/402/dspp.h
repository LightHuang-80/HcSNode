/*
 * dspp.h
 *
 *  Created on: 2020��5��27��
 *      Author: Administrator
 */

#ifndef DSPP_H_
#define DSPP_H_

#include "dsposc.h"

typedef enum DS402_PP_Status_ {
	DS402_PP_Idle,
	DS402_PP_Running,
	DS402_PP_Halt,
	DS402_PP_Error,
}DS402_PP_Status_t;

void PP_Init();
void PP_setSpeed(uint32_t speed);
void PP_OnTargetReached(int32_t position);
void PP_active(uint16_t controlWord);
uint16_t PP_exec(uint16_t controlWord);
#endif /* DSPP_H_ */
