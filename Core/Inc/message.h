/*
 * message.h
 *
 *  Created on: 2021年2月20日
 *      Author: Administrator
 */

#ifndef INC_MESSAGE_H_
#define INC_MESSAGE_H_

// Stepper driver defined
#define USE_TMCDriver 1
#define USE_TMC2209   0
#define USE_TMC2208   0
#define USE_TMC5160   1

#if (USE_TMCDriver == 1)
#include "TMCStepper.h"
#endif

typedef struct Joint_Msg {
	uint8_t type;
	uint8_t code;
	uint8_t len;
	uint8_t crc;
}Joint_Msg_t;

#define JOINT_MSG_LEN	4

typedef struct Joint_Motor_Status{
	uint32_t status;
	uint32_t tstep;
	uint16_t rms;
	uint16_t stg;
}Joint_Motor_Status_t;

typedef struct Stepper_Process_Status {
	int32_t  incsteps;

	uint16_t speed;
	uint16_t ms;

	uint16_t angle;
	int8_t   dir;
	uint8_t  state;
}Stepper_Process_Status_t;

#ifdef __cplusplus
extern "C" {
#endif

void JNT_dbgOutput(char* str, size_t cnt);
void JNT_processMsg();
void JNT_flushStepperProcessStatus();

#if (USE_TMCDriver == 1)
  void JNT_flushTMCStatus();

  #if (USE_TMC2209 == 1)
    void JNT_registerDriver(TMC2209Stepper* driver);
  #elif (USE_TMC2208 == 1)
    void JNT_registerDriver(TMC2208Stepper* driver);
  #elif (USE_TMC5160 == 1)
    void JNT_registerDriver(TMC5160Stepper* driver);
  #endif

#endif

#ifdef __cplusplus
}
#endif

#endif /* INC_MESSAGE_H_ */
