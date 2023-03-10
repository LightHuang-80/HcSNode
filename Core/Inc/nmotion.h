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
#include "PosProfile.h"
#include "PIDProfile.h"

typedef void (*pFunctTargetReach)(int32_t);

#ifdef __cplusplus
extern "C" {
#endif

typedef enum MT_eADMode_ {
	MT_ADMode_Normal,    // none accelerate
	MT_ADMode_SCurve,
	MT_ADMode_Trapezoid,
}MT_ADMode_t;

typedef enum MT_ePosQueueMode_ {
	MT_PQ_None,
	MT_PQ_SetAck,	    // Discrete motion
	MT_PQ_Queue,	    // Point queue motion
	MT_PQ_ComplexQueue, // Segments queue motion
}MT_PosQueueMode_t;

typedef enum MT_state_ {
	MT_IDLE,
	MT_START = 1,
	MT_FINISHED,
	MT_BROKEN,
	MT_TIMEOUT,
}MT_state_t;

typedef struct MT_MotionDrive_ {
	int32_t 			 target;

	int32_t				 switchPoint;
	int32_t              homePoint;

	MT_ADMode_t          adMode;	// accelerate and decelerate mode
	MT_PosQueueMode_t	 posQueueMode;

	int32_t              reachWindow;
    int32_t              win1_8;
    int32_t              win1_4;
    int32_t              win1_2;

	uint32_t             speed;

	QueueHandle_t		 msgQueue;
	QueueHandle_t        posQueue;
	QueueHandle_t        statusQueue;

	pFunctTargetReach    reachCB;

	int8_t               reportReachTarget;

	int8_t               cwDir;
	int8_t               ccwDir;
	int8_t               switchdir;
	int8_t  		     lock;      // lock to target
	int8_t               usePID;	// use pid or not
	int8_t               vel_to_cw; // positive velocity drive motor run cw or ccw

	MT_state_t           state;
}MT_MotionDrive_t;

void MT_Init(QueueHandle_t msgQueue, QueueHandle_t posQueue, QueueHandle_t statusQueue);
void MT_process_v3(uint32_t ticks);
void MT_setProfile(Node_DriveProfile_t* profile);
void MT_setReachCallback(void (*pFunct)(int32_t position));
void MT_setReachWindow(int32_t window);
void MT_setSpeed(uint32_t speed);
int32_t MT_ConvertStepsToPos(int32_t steps);
#ifdef __cplusplus
}
#endif


#endif /* INC_NMOTION_H_ */
