/*
 * nmotion.h
 *
 *  Created on: 2023年3月6日
 *      Author: Administrator
 */

#ifndef INC_NMOTION_H_
#define INC_NMOTION_H_


#include "FreeRTOS.h"
#include "queue.h"
#include "list.h"
#include "profile.h"
#include "motiondef.h"
#include "PosProfile.h"
#include "PIDProfile.h"

typedef void (*pFunctTargetReach)(int32_t);
typedef void (*pFunctMotionStartPre)();

#define REACH_OVER_COUNT   5

#ifdef __cplusplus
extern "C" {
#endif

typedef enum MT_state_ {
	MTS_Idle,
	MTS_Running = 1,
	MTS_Halting,
	MTS_Braking
}MT_state_t;

typedef struct MT_MotionDrive_ {
	MT_state_t   state;
	uint16_t     prevControlword;
	int8_t       mode;
}MT_MotionDrive_t;

void MT_init();
void MT_reset();
void MT_begin();

void MT_setMode(int8_t mode);
int8_t MT_getMode();

void MT_exec(uint16_t controlword);

void MT_loop(uint32_t ticks);
void MT_process_v3(uint32_t ticks);
void MT_setProfile(Node_DriveProfile_t* profile);
#ifdef __cplusplus
}
#endif


#endif /* INC_NMOTION_H_ */
