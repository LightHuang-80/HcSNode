/*
 * profile.h
 *
 *  Created on: 2021年2月7日
 *      Author: Administrator
 */

#ifndef INC_PROFILE_H_
#define INC_PROFILE_H_

typedef struct Node_DriveProfile{
	unsigned char motordir;   // 1 = cw, 0 = ccw
	unsigned char switchdir;  // which side the switch installed, 1 or 0
	unsigned char veltocw;    // 1 = velocity same as dir, or 0
	unsigned char homestate;  // 1 = low trigger, 0 = high trigger

	uint32_t speed;               // motor speed
	uint32_t home_speed;           // homing speed
	uint32_t home_offset;          // software zero position

	float    lower_limit_range;    // lower limit position to origin
	float    ratio;                // speed ratio
	float    acce_ratio;           // acceleration ratio

	uint16_t updatePositionDuration;   // update actual position duration
}Node_DriveProfile_t;

#endif /* INC_PROFILE_H_ */
