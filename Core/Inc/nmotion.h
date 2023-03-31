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
typedef void (*pFunctMotionStartPre)();

#define REACH_OVER_COUNT   5

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
	MTS_Idle,
	MTS_Running = 1,
	MTS_Updated,
	MTS_Halting,
	MTS_Braking
}MT_state_t;

typedef enum MREQ_state_ {
	MRS_Empty = 0,
	MRS_Running,
	MRS_Finished,
}MREQ_state_t;

typedef enum MCorrect_state_ {
	MCRT_Act,
	MCRT_Stop
}MCorrect_state;

typedef struct MT_RequestBlock_ {
	int32_t  		start_pos;
	int32_t  		target_pos;
	uint32_t 		inhibittime;
	MREQ_state_t 	state;
	int8_t          reachcounter;
	int8_t   		direction;
}MT_RequestBlock;

typedef struct MT_MotionDrive_ {
	int32_t 			 target;

	int32_t				 switchPoint;
	int32_t              homePoint;

	MT_ADMode_t          adMode;	// accelerate and decelerate mode
	int32_t              reachWindow;

	uint32_t             speed;

	QueueHandle_t		 msgQueue;
	QueueHandle_t        statusQueue;

	pFunctTargetReach    reachCB;
	pFunctMotionStartPre motionStartPreCB;

	MT_state_t           state;
	MT_RequestBlock      reqblock;

	MCorrect_state       correctingState;

	int8_t               cwDir;
	int8_t               switchdir;
	int8_t  		     lock;      // lock to target
	int8_t               vel_to_cw; // positive velocity drive motor run cw or ccw

}MT_MotionDrive_t;

void MT_Init(QueueHandle_t msgQueue);
void MT_Reset();
void MT_Begin();
void MT_Loop(uint32_t ticks);
void MT_Halt(uint32_t dece);
void MT_UpdateTask(uint8_t absolute, int32_t target, int32_t vel, uint32_t acce);
void MT_NewTask(uint8_t absolute, int32_t target, int32_t vel, uint32_t acce);
void MT_process_v3(uint32_t ticks);
void MT_setProfile(Node_DriveProfile_t* profile);
void MT_setReachCallback(void (*pFunct)(int32_t position));
void MT_setMotionStartPreCallback(void (*pFunct)());
void MT_setReachWindow(int32_t window);

int32_t MT_ConvertStepsToPos(int32_t steps);
#ifdef __cplusplus
}
#endif


#endif /* INC_NMOTION_H_ */
