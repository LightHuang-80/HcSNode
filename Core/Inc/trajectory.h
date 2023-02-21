/*
 * trajectory.h
 *
 *  Created on: 2020年11月25日
 *      Author: Administrator
 */

#ifndef SRC_MOTION_TRAJECTORY_H_
#define SRC_MOTION_TRAJECTORY_H_


void TPB_Init();
void TPB_positionUpdate(float position);
void TPB_velocityUpdate(float velocity);
void TPB_accelerationUpdate(float acceleration);
void TPB_commandSerialNumberUpdate(int32_t serialNumber);

#endif /* SRC_MOTION_TRAJECTORY_H_ */
