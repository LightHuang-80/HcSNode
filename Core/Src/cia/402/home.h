/*
 * home.h
 *
 *  Created on: 2020年10月9日
 *      Author: Administrator
 */

#ifndef SRC_CIA_402_HOME_H_
#define SRC_CIA_402_HOME_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum DS402_HOME_status_ {
	DS402_HOME_None,			// not begin the home
	DS402_HOME_SearchZeroPnt,	// do search zero point phase
	DS402_HOME_GoHome,			// touch the home switch, and go home offset
	DS402_HOME_Finished,		// reach the home offset, finish the home request
	DS402_HOME_Error,			// happen the error
}DS402_HOME_status_t;

bool      HOME_Init(int32_t offset, int8_t method, uint32_t speed, uint32_t acce);
void      HOME_setProfile(int32_t offset, int8_t method, uint32_t speed, uint32_t acce);
void      HOME_reset();
void      HOME_setFinishedCallback(void (*pfunct)(int32_t offset));
void      HOME_active();
uint16_t  HOME_exec(uint16_t controlWord);
void      HOME_finish(int32_t position);

void HOME_OnSwitchSignal(uint16_t pin);
#endif /* SRC_CIA_402_HOME_H_ */
