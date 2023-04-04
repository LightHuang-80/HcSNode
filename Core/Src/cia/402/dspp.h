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

#ifdef __cplusplus
extern "C" {
#endif

void PP_init();
void PP_active();
uint16_t PP_exec(uint16_t controlWord, uint8_t newSetPoint);
void PP_process(uint32_t ticks);
#ifdef __cplusplus
}
#endif
#endif /* DSPP_H_ */
