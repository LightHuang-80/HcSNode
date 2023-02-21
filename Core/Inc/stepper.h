/*
 * stepper.h
 *
 *  Created on: 2020年8月20日
 *      Author: Administrator
 */

#ifndef SRC_STEPPER_STEPPER_H_
#define SRC_STEPPER_STEPPER_H_

#include "planner.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STEPPER_State_Running  0x01
#define STEPPER_State_Stop     0x02
#define STEPPER_State_Acce     0x03
#define STEPPER_State_Dece     0x04

#define STEPPER_Dir_CW         1
#define STEPPER_Dir_CCW        0

#define SCURVE_MOTION_TYPE         1
#define INTERPOLATION_MOTION_TYPE  2

typedef void (*Stepper_Motion_Finished_CB)(int8_t type);

void Stepper_Init(uint8_t microsteps);

void Stepper_Run();
void Stepper_Stop();
void Stepper_RunMotion(Stepper_Motion_Finished_CB cb);

void Stepper_ChangeDir();
int8_t Stepper_GetDir();
uint32_t Stepper_GetRealSpeed();

void Stepper_RunSteps(int32_t steps);
void Stepper_RunAngle(uint16_t angle);

void Stepper_SetSpeed(uint16_t speed);
uint16_t Stepper_GetSpeed();

void Stepper_SetDir(uint32_t dir);
void Stepper_GoHome();
uint16_t Stepper_GetState();
void Stepper_Step_IRQHandle();
uint32_t Stepper_PushBlock(block_t* block);
void Stepper_ISR();

#ifdef __cplusplus
}
#endif

#endif /* SRC_STEPPER_STEPPER_H_ */
