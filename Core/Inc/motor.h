/*
 * motor.h
 *
 *  Created on: 2023年3月24日
 *      Author: Administrator
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "TMCStepper.h"
#include "nstepper.h"

#include "message.h"

#define MOTOR_DIR_FORWARD	(int8_t)1
#define MOTOR_DIR_BACKWARD	(int8_t)-1

typedef struct MOTOR_Model_ {

#if (USE_TMC5160 == 1)
	TMC5160Stepper* driver;
#else
	TMC2209Stepper* driver;
#endif

	int32_t   target;
	int32_t   speed;
	int8_t    polarity;
}MOTOR_Model_t;

#ifdef __cplusplus
extern "C" {
#endif

void MOTOR_Init();

#if (USE_TMC5160 == 1)
void MOTOR_bindDriver(TMC5160Stepper* driver);
#else
void MOTOR_bindDriver(TMC2209Stepper* driver);
#endif

void MOTOR_run();
void MOTOR_stop();
void MOTOR_loop(uint32_t ticks);

void MOTOR_setPolarity(int8_t polarity);
int8_t MOTOR_getPolarity();

void MOTOR_setDir(int8_t dir);
int8_t MOTOR_getDir();

void MOTOR_setSpeed(int32_t speed);
int32_t MOTOR_getSpeed();

void MOTOR_setRelTarget(int32_t target);
void MOTOR_setAbsTarget(int32_t target);
int32_t MOTOR_getTarget();

int32_t MOTOR_getPosDiff();

#ifdef __cplusplus
}
#endif
#endif /* INC_MOTOR_H_ */
