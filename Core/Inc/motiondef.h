/*
 * motiondef.h
 *
 *  Created on: 2023年4月3日
 *      Author: Huang
 */

#ifndef SRC_MOTION_MOTIONDEF_H_
#define SRC_MOTION_MOTIONDEF_H_

#include <stdint.h>

/* switch table 1
 * bit 7  6  5  4  3  2  1  0
 *     1                          left hardstop
 *        1                       right hardstop
 *           1                    home left switch
 *              1                 home right switch
 * */

typedef enum MREQ_state_ {
	MRS_Empty = 0,
	MRS_Running,
	MRS_Finished,
}MREQ_state_t;

/*Motion request block,
 * Homing mode, switch_touch > table 1
 * Position mode,          target_pos
 * Velocity mode,          target_vel
 * Torque mode,            target_torque
 * */
typedef struct MT_RequestBlock_ {
	MREQ_state_t 	state;

	int32_t  		target_pos;
	int32_t         target_vel;
	int32_t         target_torque;
	uint32_t 		inhibittime;
	int8_t          reachcounter;
	int8_t          switch_touch;

}MT_RequestBlock;

typedef struct MT_ResultBlock_ {
	uint32_t estimate_time;  // ms
	uint32_t use_time;       // ms
	int32_t  errcode;    // steps
}MT_ResultBlock;

typedef enum MHold_state_ {
	MHOLD_Open = 0,
	MHOLD_Close,
}MHold_state_t;

#endif /* SRC_MOTION_MOTIONDEF_H_ */
