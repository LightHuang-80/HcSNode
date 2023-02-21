/*
 * message.c
 *
 *  Created on: 2021年2月20日
 *      Author: Administrator
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "usart.h"
#include "fsusart.h"
#include "stepper.h"
#include "message.h"

// Registed driver
#if USE_TMCDriver == 1
  #include "TMCStepper.h"
  #if (USE_TMC2209 == 1)
    TMC2209Stepper* g_driver = NULL;
  #elif (USE_TMC2208 == 1)
    TMC2208Stepper* g_driver = NULL;
  #elif (USE_TMC5160 == 1)
    TMC5160Stepper* g_driver = NULL;
  #endif
#endif

// Message Type
#define MSG_SIMPLE_TYPE	  0x01
#define MSG_COMPLEX_TYPE  0x11
#define MSG_BINARY_TYPE   0x12
#define MSG_FLASH_TYPE    0x21
#define MSG_STRING_TYPE   0x22

#define MSG_FEEDBACK_TYPE 0xB3

// Simple command
#define STEPPER_RUN       0x01
#define STEPPER_STOP      0x02
#define STEPPER_CHANGEDIR 0x05
#define STEPPER_HOME      0x06

// Complex command
#define STEPPER_RUNSTEPS  0x23
#define STEPPER_DIR       0x25
#define STEPPER_GOTOANGLE 0x26
#define STEPPER_SPEED     0x27
#define STEPPER_RUNANGLE  0x28

// PID
#define STEPPER_ENPID    0x30
#define STEPPER_DISPID   0x31

// TMC
#define TMC_STALLGUARD   0x40
#define TMC_RMSCURRENT	 0x41
#define TMC_SEMIN        0x42
#define TMC_SEMAX        0x43
#define TMC_SEUP         0x44
#define TMC_SEDOWN       0x45

#define TMC_MICROSTEPS   0x47
#define TMC_TOFF         0x48
#define TMC_BLANKTIME    0x49

// Voltage PWM Mode
#define TMC_PWMAUTOSCALE 0x4A
#define TMC_PWMAUTOGRAD  0x4B
#define TMC_TPWMTHRS     0x4C
#define TMC_HSTRT        0x4D
#define TMC_HEND         0x4E
#define TMC_PWMFREQ      0x4F
#define TMC_PWMLIM       0x50
#define TMC_PWMREG       0x51
#define TMC_PWMOFS       0x52
#define TMC_PWMGRAD      0x53
#define TMC_SPREADCYCLE  0x54
#define TMC_IRUN         0x56
#define TMC_IHOLD        0x57
#define TMC_IHOLDDELAY   0x58
#define TMC_TPWRDOWN     0x59

#define TMC_FBREADY      0x60
#define TMC_FEEDBACK     0x61

#define STEPPER_FBREADY  0x80
#define STEPPER_FEEDBACK 0x81

#define TMC_STATUS         0xA0
#define STEPPER_PROCSTATUS 0xB0

#define JNT_LOGFBREADY     0xF0
#define JNT_LOGFEEDBACK    0xF1

#define JOINT_MSGSIZE 256
unsigned char g_MsgBuf[JOINT_MSGSIZE];

typedef struct Msg_Param{
	char key[12];
	union {
		int   iv;
		float fv;
	};
	uint8_t isFloat;
}Msg_Param_t;

#if (USE_TMCDriver == 1)
uint8_t  g_enTmcFeedback = 0;
uint16_t g_tmcFBFreq = 50;
#endif

uint8_t  g_enStepperFeedback = 0;
uint16_t g_stepperFBFreq = 30;

uint8_t  g_enJntLogFeedback = 0;
uint16_t g_logFBFreq = 20;

extern volatile uint16_t g_AbsAngle;
extern volatile int32_t  g_IncSteps;

void convertExpressToParam(char* str, size_t len, Msg_Param_t* param)
{
	uint8_t   exp = 0;  // express left or right
	uint8_t   idx = 0;

	char  value[16];
	memset(value, 0, 16);

	memset(param, 0, sizeof(Msg_Param_t));

	for(size_t i = 0; i < len; i++){

		if (str[i] == '=' || str[i] == ':'){
			if (exp == 0) {
			   param->key[idx] = 0;
			   idx = 0;
	           exp = 1;
			}
            continue;
		}

		if (str[i] == ' '){
			continue;
		}

		if (exp == 0){
			param->key[idx] = str[i];
		}else if (exp == 1){
			value[idx] = str[i];
			if (str[i] == '.')
				param->isFloat = true;
		}

		idx++;
	}

	if (strlen(value) <= 0){
		param->iv = 0;
	}else{
		if (!param->isFloat){
		    param->iv = atoi(value);
		}else{
			param->fv = atoff(value);
		}
	}
}

size_t parseMsg(Joint_Msg_t* msg, char *data, size_t len, Msg_Param_t* params)
{
	size_t paramsSz = 0;

	char item[64];
	int  idx = 0;

	for(size_t i = 0; i < len; i++) {
	  if(data[i] == ',' || data[i] == '|' || data[i] == ' ' || data[i] == 0x0D || data[i] == 0x13){
	    item[idx] = 0;

	    if (strlen(item) > 0){
	    	convertExpressToParam(item, idx, &params[paramsSz]);
	    	paramsSz++;
	    	idx = 0;
	    }

	    if (paramsSz >= 4)
	    	break;

	    continue;
	  }

	  item[idx] = data[i];
	  idx++;

	  if (i == len - 1){
		  // last char
		  if (strlen(item) > 0){
		    convertExpressToParam(item, idx, &params[paramsSz]);
		    paramsSz++;
		  }
	  }
	}

	return paramsSz;
}

void JNT_dbgOutput(char* str, size_t cnt)
{
	//if (g_enJntLogFeedback == 0){

	  Joint_Msg_t msg;

	  /*
	  msg.type = (uint8_t)'-'; //MSG_STRING_TYPE;
	  msg.code = (uint8_t)'-'; //JNT_LOGFEEDBACK;
	  msg.len  = (uint8_t)'-'; //cnt;
	  msg.crc  = (uint8_t)'-'; //msg.len ^ 0x1E;
      */
	  msg.type = MSG_STRING_TYPE;
	  msg.code = JNT_LOGFEEDBACK;
	  msg.len  = cnt;
	  msg.crc  = msg.len ^ 0x1E;

	  FUART_sendMsg(&msg, (uint8_t*)str, msg.len);
	//}
}

void JNT_handleSimpleMsg(Joint_Msg_t* msg)
{
	switch(msg->code){
	case STEPPER_RUN:
		Stepper_Run();
		break;
	case STEPPER_STOP:
		Stepper_Stop();
		break;
	case STEPPER_CHANGEDIR:
		Stepper_ChangeDir();
		break;
	default:
		break;
	}
}

void JNT_handleComplexMsg(Joint_Msg_t* msg, char *data, size_t len)
{
	Msg_Param_t  params[4];
    size_t paramsSz = parseMsg(msg, data, len, params);
    if (paramsSz <= 0){
    	return;
    }

    switch(msg->code){
    case STEPPER_SPEED:
    	Stepper_SetSpeed(params[0].iv);
    	break;
    case STEPPER_RUNSTEPS:
    	Stepper_RunSteps(params[0].iv);
    	break;
    case STEPPER_RUNANGLE:
    	Stepper_RunAngle(params[0].iv);
    	break;
	case STEPPER_FEEDBACK:
		g_enStepperFeedback = params[0].iv;
		if (g_enStepperFeedback == 1){
			uint16_t freq = params[1].iv;
			if (freq >= 10 && freq <= 5000){
				g_stepperFBFreq = freq;
			}

			// send back ready msg
			Joint_Msg_t fbmsg;
			fbmsg.type = MSG_BINARY_TYPE;
			fbmsg.code = STEPPER_FBREADY;
			fbmsg.len  = 2;
			fbmsg.crc  = fbmsg.len ^ 0x1E;

			uint16_t ready = 1;
			FUART_sendMsg(&fbmsg, (uint8_t*)&ready, fbmsg.len);
		}

		break;
	case JNT_LOGFEEDBACK:
		g_enJntLogFeedback = params[0].iv;
		if (g_enJntLogFeedback == 1){
			uint16_t freq = params[1].iv;
			if (freq >= 10 && freq <= 5000){
				g_logFBFreq = freq;
			}

			// send back ready msg
			Joint_Msg_t fbmsg;
			fbmsg.type = MSG_BINARY_TYPE;
			fbmsg.code = JNT_LOGFBREADY;
			fbmsg.len  = 2;
			fbmsg.crc  = fbmsg.len ^ 0x1E;

			uint16_t ready = 1;
			FUART_sendMsg(&fbmsg, (uint8_t*)&ready, fbmsg.len);
		}

		break;
    default:
    	break;
    }

#if (USE_TMCDriver == 1)
    if (g_driver != NULL){
    	switch(msg->code){
    	case TMC_MICROSTEPS:
    		g_driver->microsteps(params[0].iv);
    		break;
    	case TMC_RMSCURRENT:
    		g_driver->rms_current(params[0].iv);
    		break;
    	case TMC_STALLGUARD:
#if (USE_TMC2209 == 1)
    		g_driver->SGTHRS(params[0].iv);
#elif (USE_TMC5160 == 1)
    		g_driver->sgt(params[0].iv);
#endif
    		break;
    	case TMC_TOFF:
    		g_driver->toff(params[0].iv);
    		break;
    	case TMC_BLANKTIME:
    		g_driver->blank_time(params[0].iv);
    		break;
    	case TMC_PWMAUTOSCALE:
    		g_driver->pwm_autoscale(params[0].iv);
    		break;
    	case TMC_PWMAUTOGRAD:
    		g_driver->pwm_autograd(params[0].iv);
    		break;
    	case TMC_TPWMTHRS:
    		g_driver->TPWMTHRS(params[0].iv);
    		break;
    	case TMC_HSTRT:
    		g_driver->hysteresis_start(params[0].iv);
    		break;
    	case TMC_HEND:
    		g_driver->hysteresis_end(params[0].iv);
    		break;
    	case TMC_PWMFREQ:
    		g_driver->pwm_freq(params[0].iv);
    		break;
    	case TMC_PWMLIM:
    		g_driver->pwm_lim(params[0].iv);
    		break;
    	case TMC_PWMREG:
    		g_driver->pwm_reg(params[0].iv);
    		break;
    	case TMC_PWMOFS:
    		g_driver->pwm_ofs(params[0].iv);
    		break;
    	case TMC_PWMGRAD:
    		g_driver->pwm_grad(params[0].iv);
    		break;
    	case TMC_SPREADCYCLE:
#if (USE_TMC5160 == 1)
   			g_driver->chm(params[0].iv==0?1:0);
#endif
    		break;
    	case TMC_IRUN:
    		g_driver->irun(params[0].iv);
    		break;
    	case TMC_IHOLD:
    		g_driver->ihold(params[0].iv);
    		break;
    	case TMC_IHOLDDELAY:
    		g_driver->iholddelay(params[0].iv);
    		break;
    	case TMC_TPWRDOWN:
    		g_driver->TPOWERDOWN(params[0].iv);
    		break;
    	case TMC_FEEDBACK:
    		g_enTmcFeedback = params[0].iv;
   			if (g_enTmcFeedback == 1){
   				uint16_t freq = params[1].iv;
   				if (freq >= 10 && freq <= 5000){
   					g_tmcFBFreq = freq;
   				}

   				// send back ready msg
   				Joint_Msg_t fbmsg;
   				fbmsg.type = MSG_BINARY_TYPE;
   				fbmsg.code = TMC_FBREADY;
   				fbmsg.len  = 2;
   				fbmsg.crc  = fbmsg.len ^ 0x1E;

   				uint16_t ready = 1;
   				FUART_sendMsg(&fbmsg, (uint8_t*)&ready, fbmsg.len);
   			}

    		break;

    	default:
    		break;
    	}
    }
#endif
}

void JNT_handleFlashMsg(Joint_Msg_t* msg, char* data, size_t len)
{

}

void JNT_processMsg()
{
#if (USE_TMCDriver == 1)
	if (g_enTmcFeedback == 1){
		JNT_flushTMCStatus();
	}
#endif

	if (g_enStepperFeedback == 1){
		JNT_flushStepperProcessStatus();
	}

	if (g_enJntLogFeedback == 1){

	}

	int len = FUART_peekMsg(&huart1, g_MsgBuf);
	if (len <= 0) return;

	Joint_Msg_t *msg = (Joint_Msg_t*)g_MsgBuf;
	switch(msg->type){
	case MSG_SIMPLE_TYPE:
		JNT_handleSimpleMsg(msg);
		break;
	case MSG_COMPLEX_TYPE:
		JNT_handleComplexMsg(msg, (char*)&g_MsgBuf[JOINT_MSG_LEN], msg->len);
		break;
	case MSG_FLASH_TYPE:
		JNT_handleFlashMsg(msg, (char*)&g_MsgBuf[JOINT_MSG_LEN], msg->len);
		break;
	default:
		break;
	}
}

#if (USE_TMCDriver == 1)
void JNT_flushTMCStatus()
{
	static uint32_t tickCNT = 1;

	if (g_tmcFBFreq <= 10) return;

	if (tickCNT++ % g_tmcFBFreq == 0){
		tickCNT = 1;

		if (g_driver != NULL){
			Joint_Motor_Status_t ms;
			ms.status = g_driver->DRV_STATUS();
			ms.tstep = g_driver->TSTEP();
			ms.rms   = g_driver->rms_current();
#if (USE_TMC2209 == 1)
			ms.stg   = g_driver->SG_RESULT();
#elif (USE_TMC2208 == 1)
			ms.stg   = 0xFF;
#elif (USE_TMC5160 == 1)
			ms.stg   = g_driver->sg_result();
#endif
			Joint_Msg_t msg;
			msg.type = MSG_BINARY_TYPE;
			msg.code = TMC_STATUS;
			msg.len = sizeof(Joint_Motor_Status_t);
			msg.crc = msg.len ^ 0x1E;

			FUART_sendMsg(&msg, (uint8_t*)&ms, msg.len);
		}
	}
}
#endif

void JNT_flushStepperProcessStatus()
{
	static uint32_t tickCNT = 1;

	if (g_stepperFBFreq <= 10) return;

	if (tickCNT++ % g_stepperFBFreq == 0){
		tickCNT = 1;

		Stepper_Process_Status_t status;
		status.angle = g_AbsAngle;
		status.incsteps = g_IncSteps;
		status.speed = Stepper_GetSpeed();
		status.dir   = Stepper_GetDir();
		status.state = Stepper_GetState();

		Joint_Msg_t msg;
		msg.type = MSG_BINARY_TYPE;
		msg.code = STEPPER_PROCSTATUS;
		msg.len = sizeof(Stepper_Process_Status_t);
		msg.crc = msg.len ^ 0x1E;

		FUART_sendMsg(&msg, (uint8_t*)&status, msg.len);
	}
}

#if (USE_TMCDriver == 1)
#if (USE_TMC2209 == 1)
void JNT_registerDriver(TMC2209Stepper* driver)
{
	g_driver = driver;
}
#elif (USE_TMC2208 == 1)
void JNT_registerDriver(TMC2208Stepper* driver)
{
	g_driver = driver;
}
#elif (USE_TMC5160 == 1)
void JNT_registerDriver(TMC5160Stepper* driver)
{
	g_driver = driver;
}
#endif
#endif
